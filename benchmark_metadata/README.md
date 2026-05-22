# `benchmark_metadata/`

This folder stores JSON metadata for each generated benchmark scenario.

Each `bench_cavsXX_metadata.json` file records the design parameters and benchmark configuration for one generated Webots world, including:

- `scenario_id`;
- generated world path;
- target and actual number of surrounding CAVs;
- lane-by-lane CAV counts;
- total vehicle count;
- benchmark duration;
- warm-up duration;
- adaptive-stop settings;
- generator version.

The profiler configuration JSON files are used by `scripts/run_existing_worlds_benchmark.py` when it temporarily injects the benchmark profiler into a world file.

These metadata files are part of the reproducibility record linking each generated `.wbt` world to its benchmark settings.
