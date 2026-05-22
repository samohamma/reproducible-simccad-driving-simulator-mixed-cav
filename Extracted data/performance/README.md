# `Extracted data/performance/`

This folder contains the benchmark outputs used for the manuscript revision.

## Main files

| File | Purpose |
|---|---|
| `all_scenarios_rtf_timeseries.csv` | Consolidated timestep-level benchmark output for all `bench_cavs10` ... `bench_cavs39` scenarios. |
| `benchmark_summary.csv` | One-row-per-scenario summary of runtime outcomes and real-time-factor metrics. |
| `bench_cavsXX_rtf_summary.json` | Per-scenario JSON summary written by the profiler. |
| `rtf_figures/` | Time-series and boxplot figures generated from the benchmark CSV. |

## Important columns in `all_scenarios_rtf_timeseries.csv`

| Column | Meaning |
|---|---|
| `scenario_id` | Benchmark scenario name, e.g. `bench_cavs21`. |
| `target_surrounding_cavs` | Intended number of surrounding CAVs in the scenario. |
| `n_surrounding_cavs` | Recorded number of surrounding CAVs from metadata. |
| `n_total_vehicles` | Total vehicles including ego, broken vehicle, stop-and-go vehicles, and surrounding CAVs. |
| `cavs_lane_1` ... `cavs_lane_5` | Lane-level surrounding-CAV counts. |
| `measurement_sim_time` | Simulation time elapsed after the warm-up window. |
| `measurement_wall_time` | Wall-clock time elapsed after the warm-up window. |
| `sim_dt` | Simulation-time increment since the previous recorded row. |
| `wall_dt` | Wall-clock increment since the previous recorded row. |
| `step_real_time_factor` | Per-step diagnostic factor `sim_dt / wall_dt`; useful for diagnosis but noisy. |
| `real_time_factor` | Main elapsed metric `measurement_sim_time / measurement_wall_time`. |
| `inverse_real_time_factor` | Reciprocal of the main RTF; wall-clock seconds per simulated second. |
| `adaptive_stop_*` | Adaptive-validity-boundary settings and status. |
| `ego_x`, `ego_y`, `ego_z` | Ego vehicle position used by the adaptive-stop check. |

## Main real-time-factor definition

```text
real_time_factor = measurement_sim_time / measurement_wall_time
```

This is the primary simulation-speed metric used in the revised paper.
