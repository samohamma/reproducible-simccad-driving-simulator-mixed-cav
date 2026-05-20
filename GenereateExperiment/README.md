# `GenereateExperiment/`

This package contains the programmatic Webots world-generation layer.

It converts experiment-design parameters from `main.py` into Webots `.wbt` text. This folder is the main implementation of the paper's parameterised design-to-world-generation workflow.

## Key files

| File | Purpose |
|---|---|
| `constants.py` | Shared geometric and scenario constants. |
| `geometry.py` | Coordinate transformations and lane/road geometry helpers. |
| `road.py` | Emits road segments for on-ramp, merge, mainline, diverge, and exit sections. |
| `vehicles.py` | Emits the ego/participant vehicle, surrounding CAVs, stop-and-go vehicles, and broken vehicle. |
| `lights.py` | Emits traffic-light trigger nodes using the racing-wheel-compatible naming convention. |
| `scenery.py` | Emits optional trees/signs and other scenery objects. |
| `ground.py` | Emits ground/background objects. |
| `build.py` | Assembles the full Webots world file. |
| `profiler.py` | Helper for profiler insertion in earlier workflows; current benchmarking primarily uses `scripts/run_existing_worlds_benchmark.py`. |

## Traffic-light naming

The generated traffic-light DEF names are intentionally zero-based to remain compatible with the takeover/racing-wheel controller:

```text
RR1 -> TL_onrmp0, TL_img_StnGo0, TL_img_takeOver0, TL_offrmp0
RR2 -> TL_onrmp1, TL_img_StnGo1, TL_img_takeOver1, TL_offrmp1
```

Do not change this naming convention unless all controllers that reference traffic-light nodes are updated consistently.

## Reproducibility role

This package is where manual Webots editing is replaced by a parameterised generator. Changing benchmark parameters in `main.py` and regenerating worlds should be preferred over editing `.wbt` files by hand.
