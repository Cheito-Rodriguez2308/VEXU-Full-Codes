# Autonomous Headers

Auton code should read like a driver explaining the plan. The route files call
small actions, and the action layer handles the LemLib details.

- `auton_actions.hpp`: reusable movement and mechanism calls.
- `auton_routines.hpp`: named routine drafts.
- `auton_selector.hpp`: selected routine state.
- `paths.hpp`: LemLib path assets.

Keep this folder honest:

- Keep routes readable.
- Reuse action wrappers.
- Keep coordinates marked `TODO` until tested.

Common edits:

- Routine order and strategy.
- Poses and path names after field testing.
- Timeouts, `minSpeed`, and `earlyExitRange`.

Bring-up order:

1. Test each action alone.
2. Add timeouts before chaining.
3. Use `moveToPoint` when heading is not important.
4. Use `moveToPose` when final heading matters.
5. Use Pure Pursuit for smooth preplanned paths.

Common mistakes:

- Writing final routes before odometry is tuned.
- Turning to points too close to the robot.
- Chaining motions before accuracy is understood.
