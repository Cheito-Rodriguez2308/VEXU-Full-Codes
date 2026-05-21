# Tuning Headers

Tuning code is for repeatable checks, not route strategy. Use these tests when
the robot starts behaving differently after a repair or battery swap.

Files:

- `drive_tuning.hpp`: movement tests.
- `pid_tuning.hpp`: PID workflow explanation.
- `sensor_tuning.hpp`: sensor and tracking wheel checks.
- `tuning_menu.hpp`: controller-driven access to tests.

What belongs here:

- Keep tests small and repeatable.
- Log the result of each motion.
- Avoid final autonomous strategy.

Usually edited at practice:

- Test distances and headings after measuring the field setup.
- Which controller buttons trigger each test.
- Logging detail needed during practice.

Suggested order:

1. IMU calibrates while still.
2. Tracking wheels move positive in the expected directions.
3. Angular PID settles 90 and 180 degree turns.
4. Lateral PID settles 24 and 48 inch moves.
5. Slew reduces slip without making motion sluggish.
6. Pure Pursuit lookahead balances tightness and speed.

Common mistakes:

- Tuning multiple constants at once.
- Using integral before checking friction and backlash.
- Judging PID from one run instead of repeated runs.
