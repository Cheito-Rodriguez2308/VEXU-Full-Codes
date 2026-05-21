# Subsystem Headers

Subsystems model real mechanisms as small classes with state machines. Driver
control and auton request states; the subsystem decides what the hardware does.

Files:

- `drivetrain.hpp`: LemLib chassis wrapper and motion API.
- `intake.hpp`: intake state machine.
- `pin_mechanism.hpp`: pin manipulator state machine.
- `cup_mechanism.hpp`: cup manipulator state machine.
- `toggle_mechanism.hpp`: toggle mechanism state machine.
- `sensors.hpp`: possession and sensor facade.

What belongs here:

- Own hardware objects.
- Expose `initialize()`, `update()`, `stop()`, `setState()`, and `debug()`.
- Convert requested states into motor/pneumatic behavior.

Usually changed after mechanism testing:

- State names when real mechanism behavior is clearer.
- Motor voltages and pneumatic behavior after testing.
- Sensor thresholds for possession guards.

Common mistakes:

- Reading controller buttons inside subsystems.
- Adding final autonomous decisions inside subsystem classes.
- Skipping safety states like `Off`, `Idle`, or `Hold`.
