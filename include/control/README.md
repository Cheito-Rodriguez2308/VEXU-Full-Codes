# Control Headers

Control code turns driver input into drivetrain commands and subsystem state
requests. Subsystems should not know which button caused a request.

Main files:

- `driver_control.hpp`: reads the controller and requests actions.
- `driver_profiles.hpp`: names drive modes and input profile values.
- `input_curve.hpp`: applies driver input shaping.

What belongs here:

- Keep button mappings readable.
- Support tank, arcade, double-stick arcade, and curvature drive.
- Avoid direct mechanism motor control.

Usually edited during driver practice:

- Button preferences.
- Drive mode and precision scale.
- Deadband and curve values after driver testing.

Common mistakes:

- Binding buttons directly to motor voltage.
- Making one button do too many unrelated actions.
- Forgetting a manual override for possession guards.
