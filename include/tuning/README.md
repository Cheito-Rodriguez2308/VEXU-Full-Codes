# Tuning Headers

## Purpose

Tuning modules provide repeatable tests and notes for drivetrain, PID, sensor,
and mechanism bring-up.

## Contained Files

- `drive_tuning.hpp`: movement tests.
- `pid_tuning.hpp`: PID workflow explanation.
- `sensor_tuning.hpp`: sensor and tracking wheel checks.
- `tuning_menu.hpp`: controller-driven access to tests.

## Responsibilities

- Keep tests small and repeatable.
- Log the result of each motion.
- Avoid final autonomous strategy.

## Dependencies

Tuning depends on drivetrain, sensors, controller input, and logging.

## Students Should Modify

- Test distances and headings after measuring the field setup.
- Which controller buttons trigger each test.
- Logging detail needed during practice.

## Tuning Workflow

1. IMU calibrates while still.
2. Tracking wheels move positive in the expected directions.
3. Angular PID settles 90 and 180 degree turns.
4. Lateral PID settles 24 and 48 inch moves.
5. Slew reduces slip without making motion sluggish.
6. Pure Pursuit lookahead balances tightness and speed.

## Common Mistakes

- Tuning multiple constants at once.
- Using integral before checking friction and backlash.
- Judging PID from one run instead of repeated runs.
