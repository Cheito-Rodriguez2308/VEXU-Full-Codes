# Core Headers

## Purpose

Core headers define the robot-level owner and match state. This is the central
composition layer of the program.

## Contained Files

- `robot.hpp`: owns subsystems, driver control, autonomous, and tuning.
- `match_state.hpp`: names the current competition mode.

## Responsibilities

- Keep PROS callbacks out of subsystem code.
- Make ownership obvious.
- Keep periodic loops readable and delayed.

## Dependencies

Core depends on all team-owned modules because it wires the robot together.

## Students Should Modify

- Dashboard contents.
- Robot lifecycle sequencing.
- Which tuning or auton tools run in each mode.

## Common Mistakes

- Creating global subsystem objects outside `Robot`.
- Putting route coordinates in `Robot`.
- Forgetting to stop mechanisms in disabled mode.
