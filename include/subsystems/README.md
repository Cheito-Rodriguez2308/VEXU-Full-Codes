# Subsystem Headers

Subsystems model real mechanisms as small classes with state machines. Driver
control and auton request states; the subsystem decides what the hardware does.

- `drivetrain.hpp`: LemLib chassis wrapper and motion API.
- `intake.hpp`: intake state machine.
- `pin_mechanism.hpp`: pin manipulator state machine.
- `cup_mechanism.hpp`: cup manipulator state machine.
- `toggle_mechanism.hpp`: toggle mechanism state machine.
- `sensors.hpp`: possession and sensor facade.

Working rule: subsystems should be boring to call. The messy stuff belongs
inside the mechanism file where it can be tested without touching driver code.

Things that usually change late:

- hold voltages after the robot gets heavier
- pneumatic timing after tubing is rerouted
- possession thresholds once the intake is rebuilt
