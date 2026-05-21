# Subsystem Headers

## Purpose

Subsystems model physical robot mechanisms as small classes with state machines.

## Contained Files

- `drivetrain.hpp`: LemLib chassis wrapper and motion API.
- `intake.hpp`: intake state machine.
- `pin_mechanism.hpp`: pin manipulator state machine.
- `cup_mechanism.hpp`: cup manipulator state machine.
- `toggle_mechanism.hpp`: toggle mechanism state machine.
- `sensors.hpp`: possession and sensor facade.

## Responsibilities

- Own hardware objects.
- Expose `initialize()`, `update()`, `stop()`, `setState()`, and `debug()`.
- Convert requested states into motor/pneumatic behavior.

## Dependencies

Subsystems depend on PROS hardware types, LemLib for drivetrain motion, config,
and logging.

## Students Should Modify

- State names when real mechanism behavior is clearer.
- Motor voltages and pneumatic behavior after testing.
- Sensor thresholds for possession guards.

## Common Mistakes

- Reading controller buttons inside subsystems.
- Adding final autonomous decisions inside subsystem classes.
- Skipping safety states like `Off`, `Idle`, or `Hold`.
