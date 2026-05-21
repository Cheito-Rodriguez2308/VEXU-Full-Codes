# Source Directory

## Purpose

Implementation files for the VEX U robot scaffold live here. The folder mirrors
the team-owned folders in `include/` so students can jump from an interface to
its behavior quickly.

## Responsibilities

- Keep PROS callbacks in `main.cpp` short.
- Keep robot ownership in `core/robot.cpp`.
- Keep subsystem update logic inside subsystem files.

## Dependencies

Source files can include PROS, LemLib, and local headers. Prefer local helpers
from `util/` before duplicating logic.

## Students Should Modify

- Subsystem state transitions.
- Driver button requests.
- Autonomous routines and tuning tests.

## Common Mistakes

- Hiding robot-specific values in implementation files.
- Blocking forever without `pros::delay()`.
- Mixing autonomous route strategy into subsystem classes.
