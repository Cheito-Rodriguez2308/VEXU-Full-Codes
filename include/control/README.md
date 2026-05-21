# Control Headers

## Purpose

Control code turns driver input into drivetrain commands and subsystem state
requests.

## Contained Files

- `driver_control.hpp`: reads the controller and requests actions.
- `driver_profiles.hpp`: names drive modes and input profile values.
- `input_curve.hpp`: applies driver input shaping.

## Responsibilities

- Keep button mappings readable.
- Support tank, arcade, double-stick arcade, and curvature drive.
- Avoid direct mechanism motor control.

## Dependencies

Control depends on PROS controller types and subsystem interfaces.

## Students Should Modify

- Button preferences.
- Drive mode and precision scale.
- Deadband and curve values after driver testing.

## Common Mistakes

- Binding buttons directly to motor voltage.
- Making one button do too many unrelated actions.
- Forgetting a manual override for possession guards.
