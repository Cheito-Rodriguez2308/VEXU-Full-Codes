# Core Headers

Core owns the robot at match level. PROS calls into `Robot`, then `Robot`
updates the modules in a predictable order.

Files:

- `robot.hpp`: owns subsystems, driver control, autonomous, and tuning.
- `match_state.hpp`: names the current competition mode.

Keep this boring:

- Keep PROS callbacks out of subsystem code.
- Make ownership obvious.
- Keep periodic loops readable and delayed.

Good edits:

- Dashboard contents.
- Robot lifecycle sequencing.
- Which tuning or auton tools run in each mode.

Common mistakes:

- Creating global subsystem objects outside `Robot`.
- Putting route coordinates in `Robot`.
- Forgetting to stop mechanisms in disabled mode.
