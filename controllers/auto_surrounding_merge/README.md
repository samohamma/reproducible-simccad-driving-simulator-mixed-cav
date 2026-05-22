# `controllers/auto_surrounding_merge/`

Controller for surrounding CAVs in the merge/mainline scenario.

## Role

- Provides automated surrounding-vehicle behaviour.
- Uses longitudinal car-following logic based on the Full Velocity Difference Model (FVDM).
- Uses lateral lane-centred control based on PID-style control.
- Exchanges vehicle-state information through Webots communication devices where required.
- Logs CAV runtime data during simulation.

This is one of the main controller workloads whose computational effect is evaluated in the benchmark sweep.
