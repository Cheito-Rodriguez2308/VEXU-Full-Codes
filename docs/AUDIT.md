# Project Audit Notes

This repository is VEX U PROS C++ and LemLib code that is still in bring-up.
It is not final competition code.

## Critical Issues Checked

- PROS callback flow is isolated in `src/main.cpp`.
- `core::Robot` owns the major subsystems and match callbacks.
- LemLib `Chassis`, drivetrain, odometry sensors, and PID controller settings exist.
- Autonomous routines use action wrappers instead of embedding raw drivetrain calls everywhere.
- Nested `static/paths` assets build correctly; only `.txt` assets are packaged.

## Structural Issues Checked

- `include/` and `src/` are mirrored for config, core, subsystems, control, auton, tuning, and util.
- Required classes exist for Robot, Drivetrain, Intake, PinMechanism, CupMechanism, ToggleMechanism, Sensors, DriverControl, AutonSelector, AutonActions, tuning modules, Logger, and config.
- Robot A and Robot B share config types instead of duplicated code.

## Incomplete By Design

- Hardware ports, drivetrain measurements, PID constants, motion limits, and autonomous coordinates remain `TODO`.
- Path files in `static/paths` are examples and must be replaced with LemLib path files.
- Possession estimation uses placeholder thresholds and must be tuned with real sensors.

## Validation

- The project was cleaned and rebuilt with the bundled PROS make/toolchain.
- Build output generated both cold and hot V5 packages successfully.

## Reference Comparison

The AON Push Back repository uses clear folder grouping and concise folder-level
READMEs for orientation. The current LemLib layout stays in place, with
team-facing notes so new programmers can understand each module before editing
code.
