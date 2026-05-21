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

```text
PROS callbacks
    |
    v
core::Robot
    |
    +-- config/       Robot identity, ports, drivetrain measurements, PID
    +-- subsystems/   Drivetrain, intake, mechanisms, sensors
    +-- control/      Driver profile, input curves, button requests
    +-- auton/        Action wrappers, routine selector, path assets
    +-- tuning/       Repeatable tests and tuning workflow
    +-- util/         Logging, timers, small math helpers
```

The design intentionally favors simple ownership over clever indirection. A new
programmer should be able to answer two questions quickly: who owns this device,
and who is allowed to command it?

## Setup For New Programmers

1. Install PROS for VS Code or the PROS CLI.
2. Clone the repository.
3. Build once before editing so toolchain issues are separate from code issues.
4. Read `include/config/README.md`, then `include/core/README.md`.
5. Replace only one category of `TODO_PLACEHOLDER` values at a time.
6. Test on blocks before field testing.

## Branch Workflow

- `main` is the public portfolio branch.
- `testing` is for team bring-up, real hardware values, and route experiments.
- Merge back to `main` only when the code is clean, documented, and explainable.

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

## PID Notes

- Lateral PID controls translation error in inches using LemLib odometry.
- Angular PID controls heading error in degrees using the IMU.
- Tune `kP` until the robot reaches the target.
- Tune `kD` until overshoot and oscillation settle.
- Keep `kI` disabled unless repeated tests show real steady-state error.
- Slew limits acceleration; use it to reduce slip or tipping, not to hide bad PID.
- Exit conditions decide when a motion is "good enough" for autonomous timing.

## Common Mistakes

- Changing PID before verifying motor and sensor directions.
- Using final autonomous coordinates before odometry is trustworthy.
- Adding controller button logic inside subsystem classes.
- Leaving robot-specific values without `TODO_PLACEHOLDER`.
- Treating the public `main` branch as a testing branch.
