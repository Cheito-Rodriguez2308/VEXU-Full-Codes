# Project Audit Notes

This is our VEX U PROS C++ and LemLib code for the Override season.
Some values started from the Push Back robot, so we are checking and revising
them on the current build.

## Critical Issues Checked

- PROS callback flow is isolated in `src/main.cpp`.
- `core::Robot` owns the major subsystems and match callbacks.
- LemLib `Chassis`, drivetrain, odometry sensors, and PID controller settings exist.
- Autonomous routines use action wrappers instead of embedding raw drivetrain calls everywhere.
- Nested `static/paths` assets build correctly; only `.txt` assets are packaged.

## Structural Issues Checked

- `include/` and `src/` are mirrored for config, core, subsystems, control, auton, tuning, and util.
- Required classes exist for Robot, Drivetrain, Intake, PinMechanism,
  CupMechanism, ToggleMechanism, Sensors, DriverControl, AutonSelector,
  AutonActions, tuning modules, Logger, and config.
- Robot A and Robot B share config types instead of duplicated code.

## Needs Rechecking For Override

- A lot of starting values came from the Push Back robot, so we need to verify
  ports, drivetrain measurements, PID constants, and motion limits on the
  current Override build.
- Autonomous coordinates are not season-ready until we check them against the
  actual field setup and starting positions.
- Path files in `static/paths` need to be regenerated or retested in LemLib
  format for this robot.
- Possession estimates need a real sensor pass once the intake and object
  guides are mounted for this season.

## Validation

- The project was cleaned and rebuilt with the bundled PROS make/toolchain.
- Build output generated both cold and hot V5 packages successfully.

## Reference Comparison

The AON Push Back repo uses clear folder grouping and short folder notes. We
kept our LemLib layout in place and trimmed the notes down to what we can
actually use before editing code.
