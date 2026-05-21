# Core Headers

Core owns the robot at match level. PROS calls into `Robot`, then `Robot`
updates the modules in a predictable order.

- `robot.hpp`: owns subsystems, driver control, autonomous, and tuning.
- `match_state.hpp`: names the current competition mode.

Good place for dashboard tweaks and lifecycle order. Bad place for route
coordinates or one-off mechanism experiments.
