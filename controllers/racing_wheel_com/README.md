# `controllers/racing_wheel_com/`

Racing-wheel takeover controller.

## Role

This folder contains the compiled controller used when the participant/ego vehicle transitions from automated driving to human/racing-wheel control.

## Notes

- The working executable is retained as part of the project.
- Rebuilding this controller is not required for the benchmark workflow.
- Earlier attempts to rebuild/debug this controller required C++/OpenAL/compiler dependencies and are not part of the current publication workflow.
- Traffic-light DEF names in generated worlds must remain compatible with this controller.

Runtime CSV files in this folder are retained as controller output records.
