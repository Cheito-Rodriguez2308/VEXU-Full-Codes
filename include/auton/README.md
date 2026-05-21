# Autonomous Headers

## Purpose

Autonomous code is split into reusable actions, routine selection, routine
strategy, and path asset declarations.

## Contained Files

- `auton_actions.hpp`: reusable movement and mechanism actions.
- `auton_routines.hpp`: named scaffold routines.
- `auton_selector.hpp`: selected routine state.
- `paths.hpp`: LemLib path assets.

## Responsibilities

- Keep routes readable.
- Reuse action wrappers.
- Keep coordinates marked `TODO_PLACEHOLDER` until tested.

## Dependencies

Autonomous depends on subsystems, LemLib motion params, and static path assets.

## Students Should Modify

- Routine order and strategy.
- Placeholder poses and path names after field testing.
- Timeouts, `minSpeed`, and `earlyExitRange`.

## Tuning Workflow

1. Test each action alone.
2. Add timeouts before chaining.
3. Use `moveToPoint` when heading is not important.
4. Use `moveToPose` when final heading matters.
5. Use Pure Pursuit for smooth preplanned paths.

## Common Mistakes

- Writing final routes before odometry is tuned.
- Turning to points too close to the robot.
- Chaining motions before accuracy is understood.
