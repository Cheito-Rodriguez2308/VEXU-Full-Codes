# Config Headers

Start here when wiring changes. Most competition-day fixes should begin in
`ports.hpp` or `robot_config.hpp`, not buried in subsystem code.

Current map:

- `robot_identity.hpp`: selects SmallRobot or BigRobot.
- `ports.hpp`: groups motor, sensor, and ADI ports.
- `robot_config.hpp`: drivetrain geometry, tracking wheels, PID settings.
- `constants.hpp`: shared voltages, timeouts, tuning test targets.

Pit notes:

- negative motor ports mean reversed
- tracking wheel offset signs matter more than they look
- do not tune PID to hide a bad wheel diameter
- SmallRobot and BigRobot will not always tune the same way
