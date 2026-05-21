# Autonomous Headers

Auton is still mostly route sketches. Keep names useful, but do not pretend a
coordinate is real until it survives field tape, fresh battery, and a bumped
preload.

- `auton_actions.hpp`: reusable movement and mechanism calls.
- `auton_routines.hpp`: qual-safe ideas, risky ideas, skills chunks.
- `auton_selector.hpp`: temporary controller selector.
- `paths.hpp`: LemLib path assets from `static/paths`.

Notes from bring-up:

- test one action at a time before linking routes
- chained moves are great until odom is slightly off
- do not turn to points right beside the robot unless there is a reason
- leave slow versions in the selector during early events
