# Source Files

This folder mirrors `include/`. If a header tells you what a class can do, the
matching source file should show how it does it.

Rules we try to keep:

- Keep PROS callbacks in `main.cpp` short.
- Keep robot ownership in `core/robot.cpp`.
- Keep subsystem update logic inside subsystem files.
- Keep route decisions in `auton/`, not inside mechanisms.

Good first places to edit:

- Subsystem state transitions.
- Driver button requests.
- Autonomous routines and tuning tests.

Easy mistakes:

- Hiding robot-specific values in implementation files.
- Blocking forever without `pros::delay()`.
- Mixing autonomous route strategy into subsystem classes.
