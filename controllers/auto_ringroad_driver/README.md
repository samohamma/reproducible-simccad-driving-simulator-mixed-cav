# `controllers/auto_ringroad_driver/`

Controller for the ego/participant vehicle before takeover.

## Role

- Starts the participant vehicle in automated driving mode.
- Reads shared controller parameters through `ContrlPara.py` and `Controller_input_param.csv`.
- Uses Webots sensors including radar, GPS, receiver, emitter, display, and joystick-related interfaces.
- Handles experiment messages, in-vehicle display cues, audio cues, and transition logic toward human-driving/takeover.

## Important files

| File | Purpose |
|---|---|
| `auto_ringroad_driver.py` | Main Python controller. |
| `Display.py` / `Display.txt` | Display helper resources. |
| `Msg6.mp3` | Audio cue used by the scenario. |
| `enable_all_lidars.py` | Helper script retained with the controller. |

This controller is part of the actual experiment scenario. The benchmark profiler is separate and is only injected during benchmark runs.
