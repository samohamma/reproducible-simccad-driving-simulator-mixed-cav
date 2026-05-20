# -*- coding: utf-8 -*-
"""Performance profiler controller for automated Webots benchmark runs.

Version: v16-2026-05-15-py379-adaptive-ego-stop
Python 3.7.9 compatible.

Records every Webots basic timestep during the measurement window, but now the
measurement can terminate adaptively before the ego/ramp vehicle reaches the
on-ramp traffic light. This prevents the benchmark from measuring unintended
post-ramp/crash behaviour in low-density scenarios.

Default benchmark logic:
    warm-up: first 5 s of simulation time are excluded;
    measurement: record every timestep after warm-up;
    adaptive stop: stop when ego vehicle is within 5 m before TL_onrmp0 along
                   the world-Z direction, or when the maximum duration is reached.

Metric convention:
    step_real_time_factor = sim_dt / wall_dt
        Diagnostic per-step factor; noisy and not the main benchmark metric.

    real_time_factor = measurement_sim_time / measurement_wall_time
        Elapsed factor from the start of the measurement window to the current row.

    scenario_real_time_factor = total measurement simulation time /
                                total measurement wall time
        Final scenario-level metric written to the summary JSON/CSV.
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

from controller import Supervisor

VERSION = "v16-2026-05-15-py379-adaptive-ego-stop"
DEFAULT_CONSOLIDATED_CSV = "all_scenarios_rtf_timeseries.csv"


def _project_root():
    return os.environ.get(
        "WEBOTS_PROJECT_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
    )


def _load_json(path):
    if not path:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as exc:
        return {"_load_error": str(exc), "_path": path}


def _load_metadata(path):
    data = _load_json(path)
    if data and "_load_error" in data:
        return {"metadata_error": data.get("_load_error"), "metadata_path": path}
    return data or {}


def _as_float(value, default):
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _as_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


def _as_bool(value, default=False):
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off"):
        return False
    return bool(default)


def _finite(value):
    try:
        return isinstance(value, (int, float)) and math.isfinite(value)
    except Exception:
        return False


def _round(value, digits=6):
    if _finite(value):
        return round(value, digits)
    return value


def parse_args(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--scenario-id", default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--measurement-duration", type=float, default=None)
    parser.add_argument("--warmup-sim-time", type=float, default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--output-csv", default=None)
    parser.add_argument("--metadata", default=None)
    parser.add_argument("--adaptive-stop", dest="adaptive_stop", action="store_true", default=None)
    parser.add_argument("--no-adaptive-stop", dest="adaptive_stop", action="store_false")
    parser.add_argument("--ego-def", default=None)
    parser.add_argument("--ego-name", default=None)
    parser.add_argument("--adaptive-stop-target-def", default=None)
    parser.add_argument("--adaptive-stop-target-name", default=None)
    parser.add_argument("--adaptive-stop-before-tl-m", type=float, default=None)
    args, _ = parser.parse_known_args(argv)

    config_path = os.environ.get("WEBOTS_BENCHMARK_CONFIG")
    config = _load_json(config_path) if config_path else {}
    if config and "_load_error" in config:
        print("[performance_profiler] Could not read WEBOTS_BENCHMARK_CONFIG: {}".format(config))
        config = {}

    args.scenario_id = args.scenario_id or config.get("scenario_id") or "scenario"
    measurement_duration = args.measurement_duration
    if measurement_duration is None:
        measurement_duration = args.duration
    if measurement_duration is None:
        measurement_duration = config.get("measurement_duration_sim_s", config.get("duration", 30.0))
    args.measurement_duration = _as_float(measurement_duration, 30.0)

    warmup = args.warmup_sim_time
    if warmup is None:
        warmup = config.get("warmup_sim_time_s", 5.0)
    args.warmup_sim_time = _as_float(warmup, 5.0)

    if args.adaptive_stop is None:
        args.adaptive_stop = _as_bool(config.get("adaptive_stop", True), True)
    args.ego_def = args.ego_def or config.get("ego_def") or "EGO_PARTICIPANT"
    args.ego_name = args.ego_name or config.get("ego_name") or "veh-driver"
    args.adaptive_stop_target_def = args.adaptive_stop_target_def or config.get("adaptive_stop_target_def") or "TL_onrmp0"
    args.adaptive_stop_target_name = args.adaptive_stop_target_name or config.get("adaptive_stop_target_name") or args.adaptive_stop_target_def
    margin = args.adaptive_stop_before_tl_m
    if margin is None:
        margin = config.get("adaptive_stop_before_tl_m", 5.0)
    args.adaptive_stop_before_tl_m = _as_float(margin, 5.0)

    args.output_dir = args.output_dir or config.get("output_dir")
    args.output_csv = args.output_csv or config.get("output_csv")
    args.metadata = args.metadata or config.get("metadata")
    args.config_path = config_path
    return args


def _metadata_value(metadata, *keys):
    for key in keys:
        if key in metadata and metadata.get(key) not in (None, ""):
            return metadata.get(key)
    return ""


def _lane_count(metadata, lane):
    key = "cavs_lane_{}".format(lane)
    if key in metadata:
        return _as_int(metadata.get(key), 0)
    counts = metadata.get("surr_counts", {})
    try:
        return _as_int(counts.get(str(lane), counts.get(lane, 0)), 0)
    except Exception:
        return 0


def _node_name(node):
    try:
        field = node.getField("name")
        if field:
            return field.getSFString()
    except Exception:
        pass
    return ""


def _node_translation(node):
    if node is None:
        return None
    try:
        field = node.getField("translation")
        if field:
            return field.getSFVec3f()
    except Exception:
        return None
    return None


def _find_root_child_by_name(supervisor, name):
    if not name:
        return None
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        count = children.getCount()
        for i in range(count):
            node = children.getMFNode(i)
            if _node_name(node) == name:
                return node
    except Exception:
        pass
    return None


def _find_node(supervisor, def_name=None, name=None):
    if def_name:
        try:
            node = supervisor.getFromDef(def_name)
            if node is not None:
                return node
        except Exception:
            pass
    if name:
        return _find_root_child_by_name(supervisor, name)
    return None


def _adaptive_status(ego_node, target_node, initial_direction_sign, margin_m):
    ego_pos = _node_translation(ego_node)
    target_pos = _node_translation(target_node)
    if ego_pos is None or target_pos is None:
        return {
            "ready": False,
            "ego_x": "", "ego_y": "", "ego_z": "",
            "target_x": "", "target_y": "", "target_z": "",
            "z_gap_m": "", "xz_distance_m": "", "stop_reached": False,
        }
    ego_x, ego_y, ego_z = ego_pos[0], ego_pos[1], ego_pos[2]
    target_x, target_y, target_z = target_pos[0], target_pos[1], target_pos[2]
    z_gap = initial_direction_sign * (target_z - ego_z)
    xz_distance = math.sqrt((target_x - ego_x) ** 2 + (target_z - ego_z) ** 2)
    stop_reached = z_gap <= margin_m
    return {
        "ready": True,
        "ego_x": ego_x, "ego_y": ego_y, "ego_z": ego_z,
        "target_x": target_x, "target_y": target_y, "target_z": target_z,
        "z_gap_m": z_gap, "xz_distance_m": xz_distance, "stop_reached": stop_reached,
    }


def summarise(rows, metadata, scenario_id, measurement_duration, warmup_sim_time, basic_time_step_ms, extra_summary):
    step_rtf_values = [r["step_real_time_factor"] for r in rows if _finite(r.get("step_real_time_factor"))]
    elapsed_rtf_values = [r["real_time_factor"] for r in rows if _finite(r.get("real_time_factor"))]
    sim_dt_values = [r["sim_dt"] for r in rows if _finite(r.get("sim_dt"))]
    wall_dt_values = [r["wall_dt"] for r in rows if _finite(r.get("wall_dt"))]

    final_measurement_sim_time = rows[-1]["measurement_sim_time"] if rows else 0.0
    final_measurement_wall_time = rows[-1]["measurement_wall_time"] if rows else 0.0

    total_sim_dt = sum(sim_dt_values) if sim_dt_values else 0.0
    total_wall_dt = sum(wall_dt_values) if wall_dt_values else 0.0
    scenario_rtf = (total_sim_dt / total_wall_dt) if total_wall_dt > 0 else float("nan")
    final_elapsed_rtf = (final_measurement_sim_time / final_measurement_wall_time) if final_measurement_wall_time > 0 else float("nan")

    summary = {
        "scenario_id": scenario_id,
        "completed_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "profiler_version": VERSION,
        "warmup_sim_time_s": warmup_sim_time,
        "requested_measurement_duration_sim_s": measurement_duration,
        "basic_time_step_ms": basic_time_step_ms,
        "expected_measurement_steps_if_duration_reached": int(round(measurement_duration / (basic_time_step_ms / 1000.0))) if basic_time_step_ms else "",
        "n_samples": len(rows),
        "final_measurement_sim_time_s": final_measurement_sim_time,
        "final_measurement_wall_time_s": final_measurement_wall_time,
        "total_sim_dt_s": total_sim_dt,
        "total_wall_dt_s": total_wall_dt,
        "scenario_real_time_factor": scenario_rtf,
        "final_elapsed_real_time_factor": final_elapsed_rtf,
        "mean_elapsed_real_time_factor": (sum(elapsed_rtf_values) / len(elapsed_rtf_values)) if elapsed_rtf_values else float("nan"),
        "min_elapsed_real_time_factor": min(elapsed_rtf_values) if elapsed_rtf_values else float("nan"),
        "max_elapsed_real_time_factor": max(elapsed_rtf_values) if elapsed_rtf_values else float("nan"),
        "mean_step_real_time_factor": (sum(step_rtf_values) / len(step_rtf_values)) if step_rtf_values else float("nan"),
        "min_step_real_time_factor": min(step_rtf_values) if step_rtf_values else float("nan"),
        "max_step_real_time_factor": max(step_rtf_values) if step_rtf_values else float("nan"),
        "n_surrounding_cavs": _as_int(_metadata_value(metadata, "n_surrounding_cavs", "expected_surrounding_vehicles"), 0),
        "target_surrounding_cavs": _as_int(_metadata_value(metadata, "target_surrounding_cavs", "n_surrounding_cavs", "expected_surrounding_vehicles"), 0),
        "n_total_vehicles": _as_int(_metadata_value(metadata, "n_total_vehicles", "expected_total_vehicles"), 0),
        "cavs_lane_1": _lane_count(metadata, 1),
        "cavs_lane_2": _lane_count(metadata, 2),
        "cavs_lane_3": _lane_count(metadata, 3),
        "cavs_lane_4": _lane_count(metadata, 4),
        "cavs_lane_5": _lane_count(metadata, 5),
    }
    summary.update(extra_summary or {})
    return summary


def _append_consolidated_rows(path, fieldnames, rows):
    if not rows:
        return
    exists = os.path.exists(path) and os.path.getsize(path) > 0
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    args = parse_args(sys.argv[1:])
    root = _project_root()
    output_dir = args.output_dir or os.path.join(root, "Extracted data", "performance")
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    metadata = _load_metadata(args.metadata)
    scenario_id = args.scenario_id
    output_csv = args.output_csv or os.path.join(output_dir, DEFAULT_CONSOLIDATED_CSV)
    summary_path = os.path.join(output_dir, "{}_rtf_summary.json".format(scenario_id))

    n_surrounding_cavs = _as_int(_metadata_value(metadata, "n_surrounding_cavs", "expected_surrounding_vehicles"), 0)
    target_surrounding_cavs = _as_int(_metadata_value(metadata, "target_surrounding_cavs", "n_surrounding_cavs", "expected_surrounding_vehicles"), n_surrounding_cavs)
    n_total_vehicles = _as_int(_metadata_value(metadata, "n_total_vehicles", "expected_total_vehicles"), 0)
    cavs_lane_1 = _lane_count(metadata, 1)
    cavs_lane_2 = _lane_count(metadata, 2)
    cavs_lane_3 = _lane_count(metadata, 3)
    cavs_lane_4 = _lane_count(metadata, 4)
    cavs_lane_5 = _lane_count(metadata, 5)

    print("[performance_profiler] Starting scenario: {}".format(scenario_id))
    print("[performance_profiler] Output CSV: {}".format(output_csv))
    print("[performance_profiler] Warm-up simulation time: {} s".format(args.warmup_sim_time))
    print("[performance_profiler] Maximum measurement simulation duration: {} s".format(args.measurement_duration))
    print("[performance_profiler] Adaptive stop enabled: {}".format(args.adaptive_stop))

    robot = Supervisor()
    basic_time_step_ms = int(robot.getBasicTimeStep())
    step_ms = basic_time_step_ms

    ego_node = _find_node(robot, def_name=args.ego_def, name=args.ego_name)
    target_node = _find_node(robot, def_name=args.adaptive_stop_target_def, name=args.adaptive_stop_target_name)
    initial_direction_sign = 1.0
    adaptive_ready = False
    adaptive_note = "disabled"
    if args.adaptive_stop:
        initial_status = _adaptive_status(ego_node, target_node, initial_direction_sign, args.adaptive_stop_before_tl_m)
        if initial_status["ready"]:
            # Positive at the start means the traffic light is ahead in +Z. Negative means it is ahead in -Z.
            raw_gap = initial_status["target_z"] - initial_status["ego_z"]
            initial_direction_sign = 1.0 if raw_gap >= 0 else -1.0
            adaptive_ready = True
            adaptive_note = "ready"
        else:
            adaptive_note = "ego_or_target_not_found"
            print("[performance_profiler] WARNING: adaptive stop requested but ego/target node was not found.")
    print("[performance_profiler] Adaptive stop status: {}; ego_def={}, ego_name={}, target_def={}, margin={} m".format(adaptive_note, args.ego_def, args.ego_name, args.adaptive_stop_target_def, args.adaptive_stop_before_tl_m))

    sim_start = robot.getTime()
    wall_start = time.perf_counter()

    warmup_end_sim = float(args.warmup_sim_time)
    measurement_started = False
    measurement_sim_start = None
    measurement_wall_start = None
    prev_record_sim = None
    prev_record_wall = None
    rows = []
    stop_reason = "unknown"
    final_adaptive_status = {}

    fieldnames = [
        "scenario_id", "target_surrounding_cavs", "n_surrounding_cavs",
        "n_total_vehicles", "cavs_lane_1", "cavs_lane_2", "cavs_lane_3",
        "cavs_lane_4", "cavs_lane_5", "sample_index", "measurement_step_index",
        "sim_time", "measurement_sim_time", "wall_time", "measurement_wall_time",
        "sim_dt", "wall_dt", "step_real_time_factor", "real_time_factor",
        "inverse_real_time_factor", "basic_time_step_ms",
        "adaptive_stop_enabled", "adaptive_stop_ready", "adaptive_stop_target_def",
        "adaptive_stop_before_tl_m", "ego_x", "ego_y", "ego_z", "adaptive_stop_target_x",
        "adaptive_stop_target_y", "adaptive_stop_target_z", "adaptive_stop_z_gap_m",
        "adaptive_stop_xz_distance_m", "adaptive_stop_reached",
    ]

    while robot.step(step_ms) != -1:
        sim_now_abs = robot.getTime()
        wall_now_abs = time.perf_counter()
        elapsed_sim = sim_now_abs - sim_start
        elapsed_wall = wall_now_abs - wall_start

        adaptive_status = _adaptive_status(ego_node, target_node, initial_direction_sign, args.adaptive_stop_before_tl_m) if adaptive_ready else {}
        adaptive_stop_reached = bool(adaptive_status.get("stop_reached", False)) if adaptive_ready else False

        if (not measurement_started) and elapsed_sim >= warmup_end_sim - 1e-9:
            measurement_started = True
            measurement_sim_start = elapsed_sim
            measurement_wall_start = elapsed_wall
            prev_record_sim = measurement_sim_start
            prev_record_wall = measurement_wall_start

        if measurement_started:
            measurement_sim_time = elapsed_sim - measurement_sim_start
            if measurement_sim_time > 1e-9 and measurement_sim_time <= args.measurement_duration + 1e-9:
                measurement_wall_time = elapsed_wall - measurement_wall_start
                sim_dt = elapsed_sim - prev_record_sim
                wall_dt = elapsed_wall - prev_record_wall
                step_rtf = sim_dt / wall_dt if wall_dt > 0 else float("nan")
                elapsed_rtf = measurement_sim_time / measurement_wall_time if measurement_wall_time > 0 else float("nan")
                inverse_rtf = measurement_wall_time / measurement_sim_time if measurement_sim_time > 0 else float("nan")

                row = {
                    "scenario_id": scenario_id,
                    "target_surrounding_cavs": target_surrounding_cavs,
                    "n_surrounding_cavs": n_surrounding_cavs,
                    "n_total_vehicles": n_total_vehicles,
                    "cavs_lane_1": cavs_lane_1,
                    "cavs_lane_2": cavs_lane_2,
                    "cavs_lane_3": cavs_lane_3,
                    "cavs_lane_4": cavs_lane_4,
                    "cavs_lane_5": cavs_lane_5,
                    "sample_index": len(rows) + 1,
                    "measurement_step_index": len(rows) + 1,
                    "sim_time": round(elapsed_sim, 6),
                    "measurement_sim_time": round(measurement_sim_time, 6),
                    "wall_time": round(elapsed_wall, 6),
                    "measurement_wall_time": round(measurement_wall_time, 6),
                    "sim_dt": round(sim_dt, 6),
                    "wall_dt": round(wall_dt, 6),
                    "step_real_time_factor": _round(step_rtf, 6),
                    "real_time_factor": _round(elapsed_rtf, 6),
                    "inverse_real_time_factor": _round(inverse_rtf, 6),
                    "basic_time_step_ms": basic_time_step_ms,
                    "adaptive_stop_enabled": bool(args.adaptive_stop),
                    "adaptive_stop_ready": bool(adaptive_ready),
                    "adaptive_stop_target_def": args.adaptive_stop_target_def,
                    "adaptive_stop_before_tl_m": args.adaptive_stop_before_tl_m,
                    "ego_x": _round(adaptive_status.get("ego_x", ""), 6),
                    "ego_y": _round(adaptive_status.get("ego_y", ""), 6),
                    "ego_z": _round(adaptive_status.get("ego_z", ""), 6),
                    "adaptive_stop_target_x": _round(adaptive_status.get("target_x", ""), 6),
                    "adaptive_stop_target_y": _round(adaptive_status.get("target_y", ""), 6),
                    "adaptive_stop_target_z": _round(adaptive_status.get("target_z", ""), 6),
                    "adaptive_stop_z_gap_m": _round(adaptive_status.get("z_gap_m", ""), 6),
                    "adaptive_stop_xz_distance_m": _round(adaptive_status.get("xz_distance_m", ""), 6),
                    "adaptive_stop_reached": adaptive_stop_reached,
                }
                rows.append(row)
                prev_record_sim = elapsed_sim
                prev_record_wall = elapsed_wall
                final_adaptive_status = adaptive_status

            if adaptive_stop_reached:
                stop_reason = "adaptive_ego_before_traffic_light"
                break
            if measurement_sim_time >= args.measurement_duration - 1e-9:
                stop_reason = "duration_reached"
                break
        else:
            if adaptive_stop_reached:
                stop_reason = "adaptive_stop_before_measurement_window"
                final_adaptive_status = adaptive_status
                break

    if stop_reason == "unknown":
        stop_reason = "webots_step_returned_minus_one"

    _append_consolidated_rows(output_csv, fieldnames, rows)

    extra_summary = {
        "stop_reason": stop_reason,
        "adaptive_stop_enabled": bool(args.adaptive_stop),
        "adaptive_stop_ready": bool(adaptive_ready),
        "adaptive_stop_note": adaptive_note,
        "adaptive_stop_target_def": args.adaptive_stop_target_def,
        "adaptive_stop_target_name": args.adaptive_stop_target_name,
        "ego_def": args.ego_def,
        "ego_name": args.ego_name,
        "adaptive_stop_before_tl_m": args.adaptive_stop_before_tl_m,
        "final_ego_z": final_adaptive_status.get("ego_z", ""),
        "final_adaptive_stop_target_z": final_adaptive_status.get("target_z", ""),
        "final_adaptive_stop_z_gap_m": final_adaptive_status.get("z_gap_m", ""),
        "final_adaptive_stop_xz_distance_m": final_adaptive_status.get("xz_distance_m", ""),
        "adaptive_stop_reached": stop_reason.startswith("adaptive"),
    }
    summary = summarise(rows, metadata, scenario_id, args.measurement_duration, args.warmup_sim_time, basic_time_step_ms, extra_summary)
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("[performance_profiler] Wrote consolidated rows to: {}".format(output_csv))
    print("[performance_profiler] Wrote summary: {}".format(summary_path))
    print("[performance_profiler] Recorded {} rows for scenario {}".format(len(rows), scenario_id))
    print("[performance_profiler] Stop reason: {}".format(stop_reason))
    print("[performance_profiler] Scenario real-time factor: {}".format(summary.get("scenario_real_time_factor")))

    try:
        robot.simulationQuit(0)
    except Exception as exc:
        print("[performance_profiler] Could not call simulationQuit: {}".format(exc))


if __name__ == "__main__":
    main()
