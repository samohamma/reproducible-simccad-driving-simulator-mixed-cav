# `controllers/auto_stop_and_go/`

Controller for stop-and-go vehicles.

## Role

- Provides automated vehicles that follow stop-and-go behaviour/trajectory profiles.
- Uses `StopNGo_trajectories.csv` as the trajectory-profile input.
- Shares controller-parameter conventions with the other automated CAV controllers.

These vehicles are part of the scenario background and event structure, while the benchmark sweep primarily varies the number of surrounding CAVs.
