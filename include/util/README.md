# Utility Headers

## Purpose

Utility code holds reusable helpers that do not belong to a subsystem.

## Contained Files

- `logger.hpp`: brain, terminal, and controller-friendly logging helpers.
- `math_utils.hpp`: small math helpers.
- `timer.hpp`: simple elapsed-time helper.

## Responsibilities

- Keep helpers generic.
- Avoid robot strategy here.
- Prefer clear names over clever shortcuts.

## Dependencies

Utilities may depend on PROS timing/display APIs and LemLib pose types.

## Students Should Modify

- Log formatting.
- Extra motion samples during tuning.
- Small math helpers only when they remove duplication.

## Common Mistakes

- Turning utilities into a dumping ground.
- Adding hardware-specific behavior here.
- Logging too much during driver control.
