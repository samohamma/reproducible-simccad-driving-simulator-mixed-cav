# `controllers/`

This folder contains Webots controllers and shared controller utilities.

## Main controller groups

| Folder/file | Role |
|---|---|
| `auto_ringroad_driver/` | Ego/participant vehicle controller before takeover. |
| `racing_wheel_com/` | Racing-wheel takeover controller and related runtime outputs. |
| `auto_surrounding_merge/` | Surrounding CAV controller. |
| `auto_stop_and_go/` | Stop-and-go CAV controller using trajectory profiles. |
| `auto_broken/` | Broken/takeover-triggering vehicle controller. |
| `performance_profiler/` | Benchmark-only profiler used to record simulation-speed metrics. |
| `traffic_light_red/`, `traffic_light_green/`, `traffic_light_imaginary/` | Legacy traffic-light helper controllers. |
| `mirror/` | Mirror controller executable/source. |
| `soundPlay/` and `icon/` | Audio and display assets used by the driving scenario. |
| `Controller_input_param.csv` | Shared controller parameter file. |
| `ContrlPara.py` | Parser/helper class for controller parameters. |

## Controller logic

The surrounding vehicles are governed longitudinally using car-following logic based on the Full Velocity Difference Model (FVDM), while lateral motion is controlled to maintain lane-centred travel using PID-style control. The participant vehicle begins under automated control and can transition to the racing-wheel controller during the takeover phase.

Traffic-light nodes act as synchronisation/trigger references for the scenario, not only as visible signals. Their DEF names must remain consistent with the world-generation logic in `GenereateExperiment/lights.py`.
