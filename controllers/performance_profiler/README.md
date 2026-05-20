# `controllers/performance_profiler/`

Benchmark-only Webots Supervisor controller.

## Role

This controller is temporarily injected into a `.wbt` world by:

```bash
scripts/run_existing_worlds_benchmark.py
```

It is not part of the normal manual experiment run.

## What it records

- simulation time;
- wall-clock time;
- timestep-level increments;
- elapsed real-time factor;
- inverse real-time factor;
- benchmark scenario metadata;
- ego-vehicle position;
- adaptive-stop status.

## Main metric

```text
real_time_factor = measurement_sim_time / measurement_wall_time
```

## Adaptive stop

The profiler can stop measurement before the ego/ramp vehicle reaches a traffic-light boundary. This protects the benchmark from measuring unintended downstream/crash-prone scenario behaviour.

The default target is:

```text
TL_onrmp0
```
