# Config Headers

## Purpose

Config headers describe robot identity, ports, measured drivetrain values, PID
settings, and shared constants.

## Contained Files

- `robot_identity.hpp`: selects Robot A or Robot B.
- `ports.hpp`: groups motor, sensor, and ADI ports.
- `robot_config.hpp`: groups drivetrain, tracking, and PID settings.
- `constants.hpp`: named constants shared across the project.

## Responsibilities

- Centralize robot-specific values.
- Mark every untested value with `TODO_PLACEHOLDER`.
- Keep behavior out of config files.

## Dependencies

Config may reference PROS and LemLib types because those libraries define motor
gearsets and controller settings.

## Students Should Modify

- Ports, reversed motor signs, sensor ports.
- Track width, wheel diameter, horizontal drift.
- PID constants after tuning.

## Tuning Workflow

1. Verify ports and motor directions.
2. Verify tracking wheel directions.
3. Tune angular PID.
4. Tune lateral PID.
5. Tune slew and exit conditions.

## Common Mistakes

- Copying values from another robot.
- Changing PID before fixing sensor direction.
- Forgetting that tracking wheel offset signs matter.
