# VEX U Override LemLib Scaffold

This project is a learning and portfolio scaffold, not final competition code.
Every invented robot-specific value is marked `TODO_PLACEHOLDER` and must be
replaced, measured, tested, and explained by the student team.

## Architecture

- `src/main.cpp` is only the PROS competition callback bridge.
- `core::Robot` owns drivetrain, mechanisms, sensors, driver control, autonomous, and tuning.
- `config/` stores identity, ports, drivetrain geometry, PID values, and constants.
- `subsystems/` uses classes and state machines instead of raw controller-to-voltage bindings.
- `control/` stores driver profiles, drive modes, and input curves.
- `auton/` builds routines from reusable actions. Coordinates are placeholders.
- `tuning/` gives repeatable tests for PID, odometry, Pure Pursuit, and mechanisms.
- `util/` contains logging, math helpers, and timing helpers.

## Values that must be replaced before real robot testing

- All motor ports and reversed signs.
- Gear cartridges and external drive gear ratios.
- Drive wheel diameter, track width, drivetrain RPM, and horizontal drift.
- IMU port for Robot A and Robot B.
- Tracking wheel sensor ports, wheel size, gear ratio, offsets, and direction.
- Intake, pin, cup, toggle, and pneumatic ports.
- Optical, distance, and rotation sensor ports and thresholds.
- Controller button mapping and preferred drive mode.
- Lateral and angular PID constants.
- Slew settings, minSpeed, earlyExitRange, and timeouts.
- Autonomous starting poses and every autonomous coordinate.
- Pure Pursuit path files in `static/paths`.

## Build And Upload

Build with:

```sh
pros make
```

Upload with:

```sh
pros upload
```

Use this code on blocks or a lifted robot first. The placeholder ports are fake.

## Tuning Workflow

1. Confirm motor directions and drivetrain left/right sides from the robot point of view.
2. Calibrate the IMU on a still robot.
3. Push forward: the vertical tracking wheel should increase.
4. Push right: the horizontal tracking wheel should increase.
5. Tune angular PID with 90-degree and 180-degree turns.
6. Tune lateral PID with 24-inch and 48-inch drives.
7. Add lateral slew only after the robot drives accurately but slips or tips.
8. Test `moveToPoint` when final heading does not matter.
9. Test `moveToPose` when final heading matters.
10. Start Pure Pursuit lookahead around 10-15 inches, then adjust.
11. Use motion chaining only when speed matters more than perfect settling.
