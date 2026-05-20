# -*- coding: utf-8 -*-
"""Run/benchmark existing Webots .wbt worlds from the command line.

Version: v16-2026-05-15-py379-adaptive-ego-stop
Python 3.7.9 compatible.

Uses in-place temporary instrumentation. Records every Webots timestep after a
warm-up period into one consolidated CSV for all scenarios. Default measurement duration is 30 simulation seconds. The primary real_time_factor is the elapsed measurement_sim_time/measurement_wall_time, not the per-step value.
"""

from __future__ import print_function

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

VERSION = "v16-2026-05-15-py379-adaptive-ego-stop"
INSTRUMENT_PREFIX = "_benchmark_instrumented_"
PROFILER_MARKER = "# ---- Auto-added benchmark profiler ----"
DEFAULT_OUTPUT_SUBDIR = Path("Extracted data") / "performance"
DEFAULT_METADATA_SUBDIR = Path("benchmark_metadata")
CONSOLIDATED_CSV_NAME = "all_scenarios_rtf_timeseries.csv"


def find_project_root(start=None):
    here = Path(start or __file__).resolve()
    for candidate in [here] + list(here.parents):
        if (candidate / "worlds").is_dir() and (candidate / "controllers").is_dir():
            return candidate
    return Path(__file__).resolve().parents[1]


ROOT = find_project_root()
WORLDS_DIR = ROOT / "worlds"
OUTPUT_DIR = ROOT / DEFAULT_OUTPUT_SUBDIR
METADATA_DIR = ROOT / DEFAULT_METADATA_SUBDIR
CONSOLIDATED_CSV = OUTPUT_DIR / CONSOLIDATED_CSV_NAME


def infer_webots_home(webots_executable):
    try:
        exe = Path(webots_executable).resolve()
    except Exception:
        return None
    parts_lower = [p.lower() for p in exe.parts]
    if "webots" in parts_lower:
        idx = parts_lower.index("webots")
        return Path(*exe.parts[:idx + 1])
    if "webots.app" in parts_lower:
        idx = parts_lower.index("webots.app")
        return Path(*exe.parts[:idx + 1])
    return None


def find_webots(user_value=None):
    candidates = []
    if user_value:
        candidates.append(user_value)
    if os.environ.get("WEBOTS_EXECUTABLE"):
        candidates.append(os.environ["WEBOTS_EXECUTABLE"])
    if shutil.which("webots"):
        candidates.append(shutil.which("webots"))
    if os.environ.get("WEBOTS_HOME"):
        home = Path(os.environ["WEBOTS_HOME"])
        candidates.extend([
            str(home / "webots"),
            str(home / "webots.exe"),
            str(home / "msys64" / "mingw64" / "bin" / "webots.exe"),
        ])
    candidates.extend([
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        r"C:\Program Files\Webots\webots.exe",
        "/usr/local/webots/webots",
        "/usr/bin/webots",
        "/Applications/Webots.app/Contents/MacOS/webots",
    ])
    seen = set()
    for c in candidates:
        if not c:
            continue
        p = Path(c)
        key = str(p).lower()
        if key in seen:
            continue
        seen.add(key)
        if p.exists():
            return str(p.resolve())
    return user_value or "webots"


def slugify(value):
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")
    return text or "world"


def list_worlds(worlds_dir, recursive=False, include_instrumented=False, world_prefix=None, world_pattern=None, exclude_prefix=None):
    base = Path(worlds_dir).resolve()
    pattern = world_pattern or ("**/*.wbt" if recursive else "*.wbt")
    worlds = []
    for path in sorted(base.glob(pattern)):
        if path.name.startswith("."):
            continue
        if (not include_instrumented) and path.name.startswith(INSTRUMENT_PREFIX):
            continue
        if world_prefix and (not path.stem.startswith(world_prefix)):
            continue
        if exclude_prefix and path.stem.startswith(exclude_prefix):
            continue
        worlds.append(path.resolve())
    return worlds


def read_first_line(path):
    try:
        with open(path, "rb") as f:
            line = f.readline()
        return line.decode("utf-8", errors="replace").strip()
    except Exception as exc:
        return "<could not read: {}>".format(exc)


def quote_for_display(cmd):
    def q(x):
        s = str(x)
        return '"{}"'.format(s) if any(ch.isspace() for ch in s) else s
    return " ".join(q(x) for x in cmd)


def ensure_dirs():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    METADATA_DIR.mkdir(parents=True, exist_ok=True)


def safe_unlink(path):
    try:
        p = Path(path)
        if p.exists():
            p.unlink()
        return True, None
    except Exception as exc:
        return False, exc


def test_output_writable():
    ensure_dirs()
    p = OUTPUT_DIR / "_write_test.tmp"
    try:
        p.write_text("ok", encoding="utf-8")
        ok, err = safe_unlink(p)
        if not ok:
            return False, err
        return True, None
    except Exception as exc:
        return False, exc


def load_json_file(path):
    try:
        p = Path(path)
        if p.exists():
            return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        pass
    return {}


def source_wbproj_for_world(world_path):
    w = Path(world_path)
    candidate = w.with_name("." + w.stem + ".wbproj")
    return candidate if candidate.exists() else None


def profiler_node_text(scenario_id):
    return '''\n\n{marker}\nRobot {{\n  translation 0 0 0\n  name "benchmark_profiler_{scenario_id}"\n  controller "performance_profiler"\n  supervisor TRUE\n  synchronization TRUE\n}}\n# ---- End auto-added benchmark profiler ----\n'''.format(
        marker=PROFILER_MARKER,
        scenario_id=scenario_id,
    )


def parse_vehicle_count(world_text):
    return world_text.count("BmwX5") + world_text.count("BmwX5Au")


def make_metadata(source_world, world_text, scenario_id):
    source_world = Path(source_world)
    existing_path = METADATA_DIR / "{}_metadata.json".format(scenario_id)
    metadata = load_json_file(existing_path)
    metadata.update({
        "source_world": str(source_world.resolve()),
        "scenario_id": scenario_id,
        "world_file": source_world.name,
        "world_first_line": read_first_line(source_world),
        "world_size_bytes": source_world.stat().st_size,
        "n_vehicle_string_matches": parse_vehicle_count(world_text),
        "benchmark_version": VERSION,
        "mode_assumption": "run/Ctrl+3 unless command-line mode is changed",
        "instrumentation_method": "in-place-temporary",
    })
    if "n_surrounding_cavs" not in metadata and "expected_surrounding_vehicles" in metadata:
        metadata["n_surrounding_cavs"] = metadata.get("expected_surrounding_vehicles")
    if "n_total_vehicles" not in metadata and "expected_total_vehicles" in metadata:
        metadata["n_total_vehicles"] = metadata.get("expected_total_vehicles")
    return metadata


def write_profiler_config(scenario_id, measurement_duration, warmup_sim_time, metadata_path,
                          adaptive_stop=True, adaptive_stop_before_tl_m=5.0,
                          adaptive_stop_target_def="TL_onrmp0", adaptive_stop_target_name=None,
                          ego_def="EGO_PARTICIPANT", ego_name="veh-driver"):
    ensure_dirs()
    config_path = METADATA_DIR / "{}_profiler_config.json".format(scenario_id)
    config = {
        "scenario_id": scenario_id,
        "measurement_duration_sim_s": measurement_duration,
        "warmup_sim_time_s": warmup_sim_time,
        "output_dir": str(OUTPUT_DIR.resolve()),
        "output_csv": str(CONSOLIDATED_CSV.resolve()),
        "metadata": str(Path(metadata_path).resolve()),
        "adaptive_stop": bool(adaptive_stop),
        "adaptive_stop_before_tl_m": adaptive_stop_before_tl_m,
        "adaptive_stop_target_def": adaptive_stop_target_def,
        "adaptive_stop_target_name": adaptive_stop_target_name or adaptive_stop_target_def,
        "ego_def": ego_def,
        "ego_name": ego_name,
    }
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    return config_path.resolve()


def prepare_in_place_world(source_world, measurement_duration, warmup_sim_time,
                           adaptive_stop=True, adaptive_stop_before_tl_m=5.0,
                           adaptive_stop_target_def="TL_onrmp0", adaptive_stop_target_name=None,
                           ego_def="EGO_PARTICIPANT", ego_name="veh-driver"):
    ensure_dirs()
    source_world = Path(source_world).resolve()
    scenario_id = slugify(source_world.stem)
    original_bytes = source_world.read_bytes()
    text = original_bytes.decode("utf-8", errors="replace")
    metadata = make_metadata(source_world, text, scenario_id)
    metadata_path = METADATA_DIR / "{}_metadata.json".format(scenario_id)
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    config_path = write_profiler_config(
        scenario_id, measurement_duration, warmup_sim_time, metadata_path,
        adaptive_stop=adaptive_stop,
        adaptive_stop_before_tl_m=adaptive_stop_before_tl_m,
        adaptive_stop_target_def=adaptive_stop_target_def,
        adaptive_stop_target_name=adaptive_stop_target_name,
        ego_def=ego_def,
        ego_name=ego_name,
    )
    if PROFILER_MARKER not in text:
        text = text + profiler_node_text(scenario_id)
    source_world.write_bytes(text.encode("utf-8"))
    return {"scenario_id": scenario_id, "launch_world": source_world, "original_bytes": original_bytes, "metadata": metadata, "config_path": config_path}


def restore_in_place_world(prepared):
    if not prepared:
        return
    try:
        Path(prepared["launch_world"]).write_bytes(prepared["original_bytes"])
        print("[benchmark] Restored original world: {}".format(prepared["launch_world"]))
    except Exception as exc:
        print("[benchmark] WARNING: could not restore original world {}: {}".format(prepared.get("launch_world"), exc))


def build_webots_command(webots_executable, world_path, mode, batch=False, no_rendering=False, minimize=False, capture_output=False):
    cmd = [str(Path(webots_executable)), "--mode={}".format(mode)]
    if capture_output:
        cmd.extend(["--stdout", "--stderr"])
    if batch:
        cmd.append("--batch")
    if no_rendering:
        cmd.append("--no-rendering")
    if minimize:
        cmd.append("--minimize")
    cmd.append(str(Path(world_path).resolve()))
    return cmd


def build_environment(webots_executable, profiler_config_path=None):
    env = os.environ.copy()
    webots_home = infer_webots_home(webots_executable)
    if webots_home and webots_home.exists():
        env["WEBOTS_HOME"] = str(webots_home)
    env["WEBOTS_PROJECT_ROOT"] = str(ROOT)
    if profiler_config_path:
        env["WEBOTS_BENCHMARK_CONFIG"] = str(Path(profiler_config_path).resolve())
    # Suppress controller debug prints during benchmark runs if controllers opt in.
    env.setdefault("WEBOTS_BENCHMARK_QUIET", "1")
    return env, webots_home


def choose_cwd(webots_executable, cwd_mode):
    if cwd_mode == "project-root":
        return ROOT
    if cwd_mode == "current":
        return Path.cwd()
    try:
        exe_parent = Path(webots_executable).resolve().parent
        if exe_parent.exists():
            return exe_parent
    except Exception:
        pass
    return ROOT


def run_webots(webots_executable, world_path, mode, batch, no_rendering, minimize, timeout_s, cwd_mode, profiler_config_path=None, capture_output=False):
    cmd = build_webots_command(webots_executable, world_path, mode, batch, no_rendering, minimize, capture_output=capture_output)
    env, webots_home = build_environment(webots_executable, profiler_config_path=profiler_config_path)
    cwd = choose_cwd(webots_executable, cwd_mode)
    print("[benchmark] Launch command:")
    print(quote_for_display(cmd))
    print("[benchmark] Working directory: {}".format(cwd))
    print("[benchmark] World exists: {} -> {}".format(Path(world_path).exists(), Path(world_path).resolve()))
    if profiler_config_path:
        print("[benchmark] Profiler config: {}".format(profiler_config_path))
    if webots_home:
        print("[benchmark] WEBOTS_HOME: {}".format(webots_home))
    started = time.perf_counter()
    try:
        if capture_output:
            completed = subprocess.run(cmd, cwd=str(cwd), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=timeout_s)
        else:
            completed = subprocess.run(cmd, cwd=str(cwd), env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, universal_newlines=True, timeout=timeout_s)
            completed.stdout = ""
            completed.stderr = ""
        timed_out = False
    except subprocess.TimeoutExpired as exc:
        completed = exc
        timed_out = True
    elapsed = time.perf_counter() - started
    return {"cmd": quote_for_display(cmd), "cwd": str(cwd), "elapsed_wall_s": elapsed, "timed_out": timed_out, "returncode": None if timed_out else completed.returncode, "stdout": completed.stdout or "", "stderr": completed.stderr or ""}


def save_process_logs(scenario_id, result):
    ensure_dirs()
    stdout_path = OUTPUT_DIR / "{}_webots_stdout.log".format(scenario_id)
    stderr_path = OUTPUT_DIR / "{}_webots_stderr.log".format(scenario_id)
    stdout_path.write_text(result.get("stdout", ""), encoding="utf-8", errors="replace")
    stderr_path.write_text(result.get("stderr", ""), encoding="utf-8", errors="replace")


def load_profiler_summary(scenario_id):
    path = OUTPUT_DIR / "{}_rtf_summary.json".format(scenario_id)
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"summary_read_error": str(exc)}


def safe_write_csv(path, fieldnames, rows):
    path = Path(path)
    try:
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({k: row.get(k, "") for k in fieldnames})
        return path
    except PermissionError:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fallback = path.with_name("{}_{}{}".format(path.stem, stamp, path.suffix))
        with open(fallback, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({k: row.get(k, "") for k in fieldnames})
        print("[benchmark] WARNING: Could not write {}; wrote fallback {}".format(path, fallback))
        return fallback


def write_summary_csv(rows):
    ensure_dirs()
    out = OUTPUT_DIR / "benchmark_summary.csv"
    fieldnames = ["scenario_id", "world_file", "instrumentation_mode", "mode", "warmup_sim_time_s", "requested_measurement_duration_sim_s", "expected_measurement_steps_if_duration_reached", "process_elapsed_wall_s", "process_returncode", "process_timed_out", "summary_found", "stop_reason", "adaptive_stop_enabled", "adaptive_stop_ready", "adaptive_stop_reached", "adaptive_stop_before_tl_m", "final_ego_z", "final_adaptive_stop_target_z", "final_adaptive_stop_z_gap_m", "final_adaptive_stop_xz_distance_m", "target_surrounding_cavs", "n_surrounding_cavs", "n_total_vehicles", "cavs_lane_1", "cavs_lane_2", "cavs_lane_3", "cavs_lane_4", "cavs_lane_5", "final_measurement_sim_time_s", "final_measurement_wall_time_s", "total_sim_dt_s", "total_wall_dt_s", "scenario_real_time_factor", "final_elapsed_real_time_factor", "mean_elapsed_real_time_factor", "min_elapsed_real_time_factor", "max_elapsed_real_time_factor", "mean_step_real_time_factor", "min_step_real_time_factor", "max_step_real_time_factor", "n_samples", "cmd", "cwd"]
    return safe_write_csv(out, fieldnames, rows)


def clean_old_instrumented(worlds_dir):
    base = Path(worlds_dir)
    removed = []
    for p in sorted(base.glob(INSTRUMENT_PREFIX + "*.wbt")):
        ok, _ = safe_unlink(p)
        if ok:
            removed.append(str(p))
        safe_unlink(p.with_name("." + p.stem + ".wbproj"))
    return removed


def print_preflight(args, worlds, webots_executable):
    ok, err = test_output_writable()
    profiler = ROOT / "controllers" / "performance_profiler" / "performance_profiler.py"
    print("[preflight] Script version: {}".format(VERSION))
    print("[preflight] Python executable: {}".format(sys.executable))
    print("[preflight] Python version: {}".format(sys.version.replace(chr(10), " ")))
    print("[preflight] Project root: {}".format(ROOT))
    print("[preflight] Worlds dir: {}".format(Path(args.worlds_dir).resolve()))
    print("[preflight] Webots executable: {}".format(webots_executable))
    print("[preflight] Output dir: {} -> {}".format(OUTPUT_DIR, "WRITABLE" if ok else "NOT WRITABLE: " + str(err)))
    print("[preflight] Consolidated CSV: {}".format(CONSOLIDATED_CSV))
    print("[preflight] Profiler controller: {} -> {}".format(profiler, "FOUND" if profiler.exists() else "MISSING"))
    print("[preflight] World prefix filter: {}".format(args.world_prefix or "<none>"))
    print("[preflight] Found worlds: {}".format(len(worlds)))
    for w in worlds[:max(args.max_worlds or 10, 1)]:
        print("[preflight] - {}".format(w.name))
        print("            path: {}".format(w))
        print("            first line: {}".format(read_first_line(w)))
        print("            matching wbproj: {}".format(source_wbproj_for_world(w) or "MISSING"))


def main():
    parser = argparse.ArgumentParser(description="Benchmark existing Webots world files in Run/Ctrl+3 mode.")
    parser.add_argument("--version", action="store_true")
    parser.add_argument("--webots", default=None)
    parser.add_argument("--worlds-dir", default=str(WORLDS_DIR))
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--include-instrumented", action="store_true")
    parser.add_argument("--world-prefix", default=None)
    parser.add_argument("--exclude-prefix", default=None)
    parser.add_argument("--world-pattern", default=None)
    parser.add_argument("--duration", type=float, default=30.0, help="Measurement SIMULATION seconds after warm-up. Default is 30 s.")
    parser.add_argument("--warmup-sim-time", type=float, default=5.0)
    parser.add_argument("--no-adaptive-stop", action="store_true", help="Disable event-gated benchmark termination before the ego reaches the on-ramp traffic light.")
    parser.add_argument("--adaptive-stop-before-tl-m", type=float, default=5.0, help="Stop the benchmark when the ego vehicle is this many metres before TL_onrmp0 along world Z. Default: 5 m.")
    parser.add_argument("--adaptive-stop-target-def", default="TL_onrmp0")
    parser.add_argument("--adaptive-stop-target-name", default=None)
    parser.add_argument("--ego-def", default="EGO_PARTICIPANT")
    parser.add_argument("--ego-name", default="veh-driver")
    parser.add_argument("--sample-period", type=float, default=0.0, help="Deprecated/ignored; rows are every timestep.")
    parser.add_argument("--mode", default="run")
    parser.add_argument("--batch", action="store_true")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--minimize", action="store_true")
    parser.add_argument("--max-worlds", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--timeout-buffer", type=float, default=120.0)
    parser.add_argument("--timeout-multiplier", type=float, default=10.0)
    parser.add_argument("--cwd-mode", choices=["webots-bin", "project-root", "current"], default="webots-bin")
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--launch-source-only", action="store_true")
    parser.add_argument("--clean-instrumented", action="store_true")
    parser.add_argument("--append-results", action="store_true")
    parser.add_argument("--capture-output", action="store_true", help="Capture Webots/controller stdout/stderr logs. Off by default to reduce benchmark overhead.")
    args = parser.parse_args()

    if args.version:
        print(VERSION)
        return

    mode_alias = {"real-time": "realtime", "ctrl+2": "realtime", "ctrl+3": "run"}
    args.mode = mode_alias.get(str(args.mode).lower(), str(args.mode).lower())

    if args.clean_instrumented:
        removed = clean_old_instrumented(args.worlds_dir)
        print("[benchmark] Removed {} old instrumented .wbt files from {}".format(len(removed), Path(args.worlds_dir).resolve()))
        return

    worlds = list_worlds(args.worlds_dir, recursive=args.recursive, include_instrumented=args.include_instrumented, world_prefix=args.world_prefix, world_pattern=args.world_pattern, exclude_prefix=args.exclude_prefix)
    if args.max_worlds is not None:
        worlds = worlds[:args.max_worlds]
    webots_executable = find_webots(args.webots)

    if args.preflight:
        print_preflight(args, worlds, webots_executable)
        return
    if not worlds:
        raise SystemExit("No .wbt files found in {}".format(args.worlds_dir))

    ensure_dirs()
    if (not args.launch_source_only) and (not args.dry_run) and (not args.append_results):
        ok, err = safe_unlink(CONSOLIDATED_CSV)
        if ok:
            print("[benchmark] Reset consolidated timestep CSV: {}".format(CONSOLIDATED_CSV))
        else:
            print("[benchmark] WARNING: could not reset consolidated timestep CSV {}: {}".format(CONSOLIDATED_CSV, err))

    rows = []
    timeout_s = max((args.warmup_sim_time + args.duration) * args.timeout_multiplier + args.timeout_buffer, args.timeout_buffer)
    for source_world in worlds:
        scenario_id = slugify(source_world.stem)
        print("\n[benchmark] Preparing {}".format(source_world.stem))
        launch_world = Path(source_world).resolve()
        metadata = {"scenario_id": scenario_id, "world_file": Path(source_world).name}
        config_path = None
        prepared = None
        instrumentation_mode = "source-only" if args.launch_source_only else "in-place"
        try:
            if args.launch_source_only:
                print("[benchmark] Source world: {}".format(launch_world))
            else:
                prepared = prepare_in_place_world(
                    source_world, args.duration, args.warmup_sim_time,
                    adaptive_stop=(not args.no_adaptive_stop),
                    adaptive_stop_before_tl_m=args.adaptive_stop_before_tl_m,
                    adaptive_stop_target_def=args.adaptive_stop_target_def,
                    adaptive_stop_target_name=args.adaptive_stop_target_name,
                    ego_def=args.ego_def,
                    ego_name=args.ego_name,
                )
                launch_world = prepared["launch_world"]
                metadata = prepared["metadata"]
                config_path = prepared["config_path"]
                print("[benchmark] In-place temporary instrumentation: {}".format(launch_world))
            if args.dry_run:
                result = {"cmd": quote_for_display(build_webots_command(webots_executable, launch_world, args.mode, args.batch, args.no_rendering, args.minimize, capture_output=args.capture_output)), "cwd": str(choose_cwd(webots_executable, args.cwd_mode)), "elapsed_wall_s": 0, "timed_out": False, "returncode": "DRY_RUN"}
            else:
                result = run_webots(webots_executable, launch_world, args.mode, args.batch, args.no_rendering, args.minimize, timeout_s, args.cwd_mode, profiler_config_path=config_path, capture_output=args.capture_output)
                if args.capture_output:
                    save_process_logs(scenario_id, result)
            profiler_summary = {} if args.launch_source_only else load_profiler_summary(scenario_id)
            rows.append({
                "scenario_id": scenario_id,
                "world_file": Path(source_world).name,
                "instrumentation_mode": instrumentation_mode,
                "mode": args.mode,
                "warmup_sim_time_s": args.warmup_sim_time,
                "requested_measurement_duration_sim_s": args.duration,
                "expected_measurement_steps_if_duration_reached": profiler_summary.get("expected_measurement_steps_if_duration_reached", profiler_summary.get("expected_measurement_steps", "")),
                "process_elapsed_wall_s": round(result.get("elapsed_wall_s", 0), 3),
                "process_returncode": result.get("returncode"),
                "process_timed_out": result.get("timed_out"),
                "summary_found": bool(profiler_summary),
                "stop_reason": profiler_summary.get("stop_reason", ""),
                "adaptive_stop_enabled": profiler_summary.get("adaptive_stop_enabled", (not args.no_adaptive_stop)),
                "adaptive_stop_ready": profiler_summary.get("adaptive_stop_ready", ""),
                "adaptive_stop_reached": profiler_summary.get("adaptive_stop_reached", ""),
                "adaptive_stop_before_tl_m": profiler_summary.get("adaptive_stop_before_tl_m", args.adaptive_stop_before_tl_m),
                "final_ego_z": profiler_summary.get("final_ego_z", ""),
                "final_adaptive_stop_target_z": profiler_summary.get("final_adaptive_stop_target_z", ""),
                "final_adaptive_stop_z_gap_m": profiler_summary.get("final_adaptive_stop_z_gap_m", ""),
                "final_adaptive_stop_xz_distance_m": profiler_summary.get("final_adaptive_stop_xz_distance_m", ""),
                "target_surrounding_cavs": profiler_summary.get("target_surrounding_cavs", metadata.get("target_surrounding_cavs", metadata.get("n_surrounding_cavs", metadata.get("expected_surrounding_vehicles", "")))),
                "n_surrounding_cavs": profiler_summary.get("n_surrounding_cavs", metadata.get("n_surrounding_cavs", metadata.get("expected_surrounding_vehicles", ""))),
                "n_total_vehicles": profiler_summary.get("n_total_vehicles", metadata.get("n_total_vehicles", metadata.get("expected_total_vehicles", ""))),
                "cavs_lane_1": profiler_summary.get("cavs_lane_1", metadata.get("cavs_lane_1", "")),
                "cavs_lane_2": profiler_summary.get("cavs_lane_2", metadata.get("cavs_lane_2", "")),
                "cavs_lane_3": profiler_summary.get("cavs_lane_3", metadata.get("cavs_lane_3", "")),
                "cavs_lane_4": profiler_summary.get("cavs_lane_4", metadata.get("cavs_lane_4", "")),
                "cavs_lane_5": profiler_summary.get("cavs_lane_5", metadata.get("cavs_lane_5", "")),
                "final_measurement_sim_time_s": profiler_summary.get("final_measurement_sim_time_s", ""),
                "final_measurement_wall_time_s": profiler_summary.get("final_measurement_wall_time_s", ""),
                "total_sim_dt_s": profiler_summary.get("total_sim_dt_s", ""),
                "total_wall_dt_s": profiler_summary.get("total_wall_dt_s", ""),
                "scenario_real_time_factor": profiler_summary.get("scenario_real_time_factor", ""),
                "final_elapsed_real_time_factor": profiler_summary.get("final_elapsed_real_time_factor", ""),
                "mean_elapsed_real_time_factor": profiler_summary.get("mean_elapsed_real_time_factor", ""),
                "min_elapsed_real_time_factor": profiler_summary.get("min_elapsed_real_time_factor", ""),
                "max_elapsed_real_time_factor": profiler_summary.get("max_elapsed_real_time_factor", ""),
                "mean_step_real_time_factor": profiler_summary.get("mean_step_real_time_factor", ""),
                "min_step_real_time_factor": profiler_summary.get("min_step_real_time_factor", ""),
                "max_step_real_time_factor": profiler_summary.get("max_step_real_time_factor", ""),
                "n_samples": profiler_summary.get("n_samples", ""),
                "cmd": result.get("cmd", ""),
                "cwd": result.get("cwd", ""),
            })
        finally:
            if prepared is not None:
                restore_in_place_world(prepared)
    summary_csv = write_summary_csv(rows)
    print("\n[benchmark] Wrote summary: {}".format(summary_csv))
    print("[benchmark] Consolidated timestep CSV: {}".format(CONSOLIDATED_CSV))
    print("[benchmark] Output folder: {}".format(OUTPUT_DIR))


if __name__ == "__main__":
    main()
