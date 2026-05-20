# -*- coding: utf-8 -*-
"""Unified entry point for Webots experiment generation and benchmarking.

Default behaviour:
    python main.py
        Generates worlds/ExperimentDesign.wbt only.

Benchmark workflow:
    python main.py --benchmark --mode run
        Generates benchmark worlds for every total surrounding-CAV count from
        10 to 39, changing ONLY the number of surrounding CAVs per lane, then
        benchmarks them in Webots Run/Ctrl+3 mode. Default measurement duration
        is a maximum of 30 simulation seconds after a 5 second warm-up. The
        benchmark now stops adaptively if the ego/ramp vehicle reaches 5 m
        before the on-ramp traffic light.

Python 3.7.9 compatible.
"""

from __future__ import print_function

import argparse
import json
import subprocess
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
WORLDS_DIR = THIS_DIR / "worlds"
METADATA_DIR = THIS_DIR / "benchmark_metadata"
PERFORMANCE_DIR = THIS_DIR / "Extracted data" / "performance"

for directory in (WORLDS_DIR, METADATA_DIR, PERFORMANCE_DIR):
    if not directory.exists():
        directory.mkdir(parents=True)

if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from GenereateExperiment import make_roads, build_world

VERSION = "v16-2026-05-15-adaptive-ego-stop-cavs10-39-py379"
BENCHMARK_PREFIX = "bench_"

BASELINE_EXPERIMENT = {
    "scenario_id": "ExperimentDesign",
    "N_road": 2,
    "n_lanes": 5,
    "L_mid": 700.0,
    "L_ramp": [1000.0, 800.0],
    "L_main": 12300.0,
    "surr_counts": {1: 12, 2: 10, 3: 6, 4: 1, 5: 0},
    "include_trees": True,
    "include_signs": True,
    "seed": 42,
}

BENCHMARK_CAV_TOTAL_MIN = 10
BENCHMARK_CAV_TOTAL_MAX = 39
BENCHMARK_ACTIVE_SURROUNDING_LANES = [1, 2, 3, 4]


def allocate_surrounding_cavs(total_cavs, lane_weights=None):
    """Allocate a total number of surrounding CAVs across the four active lanes.

    The allocation preserves the baseline density pattern as closely as possible
    while producing integer lane counts that sum exactly to ``total_cavs``.
    Lane 5 remains zero because the current experiment uses four surrounding
    lanes for this benchmark sweep.
    """
    total_cavs = int(total_cavs)
    if total_cavs < 0:
        raise ValueError("total_cavs must be non-negative")
    weights = dict(lane_weights or {1: 12, 2: 10, 3: 6, 4: 1})
    lanes = list(BENCHMARK_ACTIVE_SURROUNDING_LANES)
    weight_sum = float(sum(float(weights.get(lane, 0.0)) for lane in lanes))
    if weight_sum <= 0:
        base = total_cavs // len(lanes)
        counts = {lane: base for lane in lanes}
        for lane in lanes[:total_cavs - sum(counts.values())]:
            counts[lane] += 1
    else:
        raw = {lane: total_cavs * float(weights.get(lane, 0.0)) / weight_sum for lane in lanes}
        counts = {lane: int(raw[lane]) for lane in lanes}
        remainder = total_cavs - sum(counts.values())
        order = sorted(lanes, key=lambda lane: (raw[lane] - int(raw[lane]), -lane), reverse=True)
        for lane in order[:remainder]:
            counts[lane] += 1
    counts[5] = 0
    return counts


def build_default_benchmark_scenarios(min_total=BENCHMARK_CAV_TOTAL_MIN, max_total=BENCHMARK_CAV_TOTAL_MAX):
    scenarios = []
    for total in range(int(min_total), int(max_total) + 1):
        scenarios.append({
            "scenario_id": "bench_cavs{}".format(total),
            "surr_counts": allocate_surrounding_cavs(total),
        })
    return scenarios


DEFAULT_BENCHMARK_SCENARIOS = build_default_benchmark_scenarios()

ALLOWED_BENCHMARK_KEYS = set(["scenario_id", "surr_counts"])


def _int_key_counts(counts):
    return {int(k): int(v) for k, v in dict(counts).items()}


def _copy_baseline():
    scenario = dict(BASELINE_EXPERIMENT)
    scenario["L_ramp"] = list(BASELINE_EXPERIMENT["L_ramp"])
    scenario["surr_counts"] = dict(BASELINE_EXPERIMENT["surr_counts"])
    return scenario


def normalise_benchmark_scenario(raw):
    raw = dict(raw)
    ignored = sorted([k for k in raw.keys() if k not in ALLOWED_BENCHMARK_KEYS])
    if ignored:
        print("[main] WARNING: benchmark scenario '{}' contains ignored keys: {}".format(raw.get("scenario_id", "<unnamed>"), ", ".join(ignored)))
        print("[main]          Only 'scenario_id' and 'surr_counts' are allowed to vary.")
    scenario = _copy_baseline()
    scenario["scenario_id"] = raw.get("scenario_id", "benchmark")
    scenario["surr_counts"] = _int_key_counts(raw.get("surr_counts", BASELINE_EXPERIMENT["surr_counts"]))
    return scenario


def expected_total_vehicles(scenario):
    counts = _int_key_counts(scenario.get("surr_counts", {}))
    n_lanes = int(scenario.get("n_lanes", BASELINE_EXPERIMENT["n_lanes"]))
    surrounding = sum(counts.values())
    stopgo = 2 * max(0, min(4, n_lanes - 1))
    broken = 1
    participant = 1
    return surrounding + stopgo + broken + participant


def load_benchmark_scenarios(config_path=None):
    if not config_path:
        return list(DEFAULT_BENCHMARK_SCENARIOS)
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict):
        scenarios = data.get("scenarios", [])
    elif isinstance(data, list):
        scenarios = data
    else:
        raise ValueError("Benchmark config must be a list or an object with a 'scenarios' list.")
    if not scenarios:
        raise ValueError("Benchmark config contains no scenarios.")
    return scenarios


def scenario_to_roads(scenario):
    return make_roads(
        int(scenario.get("N_road", BASELINE_EXPERIMENT["N_road"])),
        n_lanes=int(scenario.get("n_lanes", BASELINE_EXPERIMENT["n_lanes"])),
        L_mid=float(scenario.get("L_mid", BASELINE_EXPERIMENT["L_mid"])),
        L_ramp=scenario.get("L_ramp", BASELINE_EXPERIMENT["L_ramp"]),
        L_main=float(scenario.get("L_main", BASELINE_EXPERIMENT["L_main"])),
        surr_counts_rr1=_int_key_counts(scenario.get("surr_counts", BASELINE_EXPERIMENT["surr_counts"])),
    )


def write_metadata(scenario, world_path, benchmark_duration=None, warmup_sim_time=None,
                   adaptive_stop=True, adaptive_stop_before_tl_m=5.0,
                   adaptive_stop_target_def="TL_onrmp0", ego_def="EGO_PARTICIPANT",
                   ego_name="veh-driver"):
    scenario_id = scenario.get("scenario_id", Path(world_path).stem)
    n_surrounding_cavs = sum(_int_key_counts(scenario.get("surr_counts", {})).values())
    n_total_vehicles = expected_total_vehicles(scenario)
    metadata = dict(scenario)
    counts = _int_key_counts(scenario.get("surr_counts", {}))
    metadata.update({
        "scenario_id": scenario_id,
        "world_path": str(Path(world_path).resolve()),
        "n_surrounding_cavs": n_surrounding_cavs,
        "target_surrounding_cavs": n_surrounding_cavs,
        "n_total_vehicles": n_total_vehicles,
        "expected_surrounding_vehicles": n_surrounding_cavs,
        "expected_total_vehicles": n_total_vehicles,
        "benchmark_varied_parameter": "total_surrounding_cavs_10_to_39",
        "benchmark_allocation_method": "baseline_weighted_largest_remainder_across_lanes_1_to_4",
        "cavs_lane_1": counts.get(1, 0),
        "cavs_lane_2": counts.get(2, 0),
        "cavs_lane_3": counts.get(3, 0),
        "cavs_lane_4": counts.get(4, 0),
        "cavs_lane_5": counts.get(5, 0),
        "generated_by": "main.py",
        "generator_version": VERSION,
        "profiler_embedded_in_world": False,
        "adaptive_stop": bool(adaptive_stop),
        "adaptive_stop_before_tl_m": adaptive_stop_before_tl_m,
        "adaptive_stop_target_def": adaptive_stop_target_def,
        "ego_def": ego_def,
        "ego_name": ego_name,
    })
    if benchmark_duration is not None:
        metadata["measurement_duration_sim_s"] = benchmark_duration
    if warmup_sim_time is not None:
        metadata["warmup_sim_time_s"] = warmup_sim_time
    metadata_path = METADATA_DIR / "{}_metadata.json".format(scenario_id)
    with open(str(metadata_path), "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)
    return metadata_path


def generate_world(scenario, output_name=None, benchmark_duration=None, warmup_sim_time=None,
                   adaptive_stop=True, adaptive_stop_before_tl_m=5.0,
                   adaptive_stop_target_def="TL_onrmp0", ego_def="EGO_PARTICIPANT",
                   ego_name="veh-driver"):
    scenario_id = scenario.get("scenario_id", "ExperimentDesign")
    world_name = output_name or "{}.wbt".format(scenario_id)
    world_path = WORLDS_DIR / world_name
    roads = scenario_to_roads(scenario)
    world_text = build_world(road_cfgs=roads, include_trees=bool(scenario.get("include_trees", True)), include_signs=bool(scenario.get("include_signs", True)), seed=int(scenario.get("seed", 42)), include_performance_profiler=False, profiler_config=None)
    with open(str(world_path), "w", encoding="utf-8") as f:
        f.write(world_text)
    metadata_path = write_metadata(
        scenario, world_path, benchmark_duration=benchmark_duration,
        warmup_sim_time=warmup_sim_time, adaptive_stop=adaptive_stop,
        adaptive_stop_before_tl_m=adaptive_stop_before_tl_m,
        adaptive_stop_target_def=adaptive_stop_target_def, ego_def=ego_def, ego_name=ego_name,
    )
    print("[main] Wrote world: {}".format(world_path))
    print("[main] Wrote metadata: {}".format(metadata_path))
    return world_path


def clean_generated_benchmark_worlds(prefix=BENCHMARK_PREFIX):
    removed = []
    for path in sorted(WORLDS_DIR.glob("{}*.wbt".format(prefix))):
        try:
            path.unlink()
            removed.append(str(path))
        except Exception as exc:
            print("[main] WARNING: could not remove {}: {}".format(path, exc))
    for path in sorted(WORLDS_DIR.glob("_benchmark_instrumented_*.wbt")):
        try:
            path.unlink()
        except Exception:
            pass
    print("[main] Removed {} generated benchmark world(s).".format(len(removed)))


def generate_baseline_world():
    return generate_world(BASELINE_EXPERIMENT, output_name="ExperimentDesign.wbt")


def generate_benchmark_worlds(config_path=None, max_scenarios=None, clean_first=True, benchmark_duration=30.0, warmup_sim_time=5.0,
                              adaptive_stop=True, adaptive_stop_before_tl_m=5.0,
                              adaptive_stop_target_def="TL_onrmp0", ego_def="EGO_PARTICIPANT",
                              ego_name="veh-driver"):
    if clean_first:
        clean_generated_benchmark_worlds(BENCHMARK_PREFIX)
    scenarios = load_benchmark_scenarios(config_path)
    if max_scenarios is not None:
        scenarios = scenarios[:max_scenarios]
    print("[main] Benchmark sweep varies ONLY total surrounding CAVs across lanes 1-4.")
    print("[main] Default sweep: {} to {} surrounding CAVs; maximum measurement duration: {} s.".format(BENCHMARK_CAV_TOTAL_MIN, BENCHMARK_CAV_TOTAL_MAX, benchmark_duration))
    print("[main] Adaptive benchmark stop: {}; target={}, margin={} m.".format(bool(adaptive_stop), adaptive_stop_target_def, adaptive_stop_before_tl_m))
    generated = []
    for raw in scenarios:
        scenario = normalise_benchmark_scenario(raw)
        scenario_id = scenario.get("scenario_id", "benchmark")
        if not str(scenario_id).startswith(BENCHMARK_PREFIX):
            scenario_id = BENCHMARK_PREFIX + str(scenario_id)
            scenario["scenario_id"] = scenario_id
        generated.append(generate_world(
            scenario, output_name="{}.wbt".format(scenario_id),
            benchmark_duration=benchmark_duration, warmup_sim_time=warmup_sim_time,
            adaptive_stop=adaptive_stop, adaptive_stop_before_tl_m=adaptive_stop_before_tl_m,
            adaptive_stop_target_def=adaptive_stop_target_def, ego_def=ego_def, ego_name=ego_name,
        ))
    return generated


def run_benchmarks(duration=30.0, warmup_sim_time=5.0, mode="run", max_worlds=None, webots=None, batch=False, no_rendering=False, minimize=False, cwd_mode="webots-bin", world_prefix=BENCHMARK_PREFIX, append_results=False, capture_output=False, no_adaptive_stop=False, adaptive_stop_before_tl_m=5.0, adaptive_stop_target_def="TL_onrmp0", ego_def="EGO_PARTICIPANT", ego_name="veh-driver"):
    script = THIS_DIR / "scripts" / "run_existing_worlds_benchmark.py"
    if not script.exists():
        raise RuntimeError("Benchmark runner not found: {}".format(script))
    cmd = [sys.executable, str(script), "--mode", mode, "--duration", str(duration), "--warmup-sim-time", str(warmup_sim_time), "--world-prefix", world_prefix, "--cwd-mode", cwd_mode, "--adaptive-stop-before-tl-m", str(adaptive_stop_before_tl_m), "--adaptive-stop-target-def", adaptive_stop_target_def, "--ego-def", ego_def, "--ego-name", ego_name]
    if max_worlds is not None:
        cmd.extend(["--max-worlds", str(max_worlds)])
    if webots:
        cmd.extend(["--webots", webots])
    if batch:
        cmd.append("--batch")
    if no_rendering:
        cmd.append("--no-rendering")
    if minimize:
        cmd.append("--minimize")
    if no_adaptive_stop:
        cmd.append("--no-adaptive-stop")
    if append_results:
        cmd.append("--append-results")
    if capture_output:
        cmd.append("--capture-output")
    print("[main] Running benchmark command:")
    print("[main] {}".format(" ".join('"{}"'.format(x) if " " in str(x) else str(x) for x in cmd)))
    return subprocess.call(cmd, cwd=str(THIS_DIR))


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Generate Webots experiment worlds and optionally benchmark them.")
    parser.add_argument("--version", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--benchmark-generate-only", action="store_true")
    parser.add_argument("--config", default=None, help="Optional JSON benchmark config. Only scenario_id and surr_counts are used; defaults generate total surrounding CAVs from 10 to 39.")
    parser.add_argument("--duration", type=float, default=30.0, help="Measurement SIMULATION seconds after warm-up. Default is 30 s.")
    parser.add_argument("--warmup-sim-time", type=float, default=5.0)
    parser.add_argument("--no-adaptive-stop", action="store_true", help="Disable event-gated stopping before the ego/ramp vehicle reaches the on-ramp traffic light.")
    parser.add_argument("--adaptive-stop-before-tl-m", type=float, default=5.0, help="Stop when the ego is this many metres before TL_onrmp0 along world Z. Default: 5 m.")
    parser.add_argument("--adaptive-stop-target-def", default="TL_onrmp0")
    parser.add_argument("--ego-def", default="EGO_PARTICIPANT")
    parser.add_argument("--ego-name", default="veh-driver")
    parser.add_argument("--sample-period", type=float, default=0.0, help="Deprecated/ignored; rows are every timestep.")
    parser.add_argument("--mode", default="run")
    parser.add_argument("--max-worlds", type=int, default=None)
    parser.add_argument("--max-scenarios", type=int, default=None)
    parser.add_argument("--webots", default=None)
    parser.add_argument("--no-clean", action="store_true")
    parser.add_argument("--append-results", action="store_true")
    parser.add_argument("--capture-output", action="store_true", help="Capture Webots/controller stdout/stderr logs. Off by default to reduce benchmark overhead.")
    parser.add_argument("--batch", action="store_true")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--minimize", action="store_true")
    parser.add_argument("--cwd-mode", choices=["webots-bin", "project-root", "current"], default="webots-bin")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if args.version:
        print(VERSION)
        return 0
    mode_alias = {"real-time": "realtime", "ctrl+2": "realtime", "ctrl+3": "run"}
    args.mode = mode_alias.get(str(args.mode).lower(), str(args.mode).lower())
    generate_baseline_world()
    if args.benchmark or args.benchmark_generate_only:
        generated = generate_benchmark_worlds(
            config_path=args.config, max_scenarios=args.max_scenarios, clean_first=(not args.no_clean),
            benchmark_duration=args.duration, warmup_sim_time=args.warmup_sim_time,
            adaptive_stop=(not args.no_adaptive_stop),
            adaptive_stop_before_tl_m=args.adaptive_stop_before_tl_m,
            adaptive_stop_target_def=args.adaptive_stop_target_def,
            ego_def=args.ego_def, ego_name=args.ego_name,
        )
        print("[main] Generated {} benchmark world(s).".format(len(generated)))
    if args.benchmark and not args.benchmark_generate_only:
        return run_benchmarks(
            duration=args.duration, warmup_sim_time=args.warmup_sim_time, mode=args.mode,
            max_worlds=args.max_worlds, webots=args.webots, batch=args.batch,
            no_rendering=args.no_rendering, minimize=args.minimize, cwd_mode=args.cwd_mode,
            world_prefix=BENCHMARK_PREFIX, append_results=args.append_results,
            capture_output=args.capture_output, no_adaptive_stop=args.no_adaptive_stop,
            adaptive_stop_before_tl_m=args.adaptive_stop_before_tl_m,
            adaptive_stop_target_def=args.adaptive_stop_target_def,
            ego_def=args.ego_def, ego_name=args.ego_name,
        )
    print("[main] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
