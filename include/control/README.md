# Control Headers

Control code turns driver input into drivetrain commands and subsystem state
requests. Subsystems should not know which button caused a request.

Files:

- `driver_control.hpp`: reads the controller and requests actions.
- `driver_profiles.hpp`: names drive modes and input profile values.
- `input_curve.hpp`: applies driver input shaping.

Driver notes:

- button mapping will probably change after the first full driver practice
- keep the manual override obvious
- precision mode is useful for testing even if drivers do not keep it
- avoid combo buttons until the base controls feel boring
