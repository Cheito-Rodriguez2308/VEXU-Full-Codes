# Config Headers

Start here when wiring, sensors, or tuning values change.

Main files:

- `robot_identity.hpp`: selects Robot A or Robot B.
- `ports.hpp`: groups motor, sensor, and ADI ports.
- `robot_config.hpp`: groups drivetrain, tracking, and PID settings.
- `constants.hpp`: named constants shared across the project.

What belongs here:

- Centralize robot-specific values.
- Mark every untested value with `TODO`.
- Keep behavior out of config files.

Config can reference PROS and LemLib types because those libraries define motor
gearsets and controller settings. Keep the actual driving logic somewhere else.

Change these first:

- Ports, reversed motor signs, sensor ports.
- Track width, wheel diameter, horizontal drift.
- PID constants after tuning.

Bring-up order:

1. Verify ports and motor directions.
2. Verify tracking wheel directions.
3. Tune angular PID.
4. Tune lateral PID.
5. Tune slew and exit conditions.

Common mistakes:

- Copying values from another robot.
- Changing PID before fixing sensor direction.
- Forgetting that tracking wheel offset signs matter.
