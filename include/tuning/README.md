# Tuning Headers

Tuning code is our quick checkup kit. Run these when the robot starts acting
different after a repair, battery swap, or wheel change.

Order we usually want:

1. IMU calibrates while still.
2. Tracking wheels move positive in the expected directions.
3. Angular PID settles 90 and 180 degree turns.
4. Lateral PID settles 24 and 48 inch moves.
5. Slew reduces slip without making motion sluggish.
6. Pure Pursuit lookahead balances tightness and speed.

Keep notes after runs:

- battery voltage
- tile vs. board
- whether the robot had game objects loaded
- what was changed since the last run
