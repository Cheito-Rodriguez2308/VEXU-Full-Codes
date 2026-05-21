# Utility Headers

Utilities are small helpers shared by several modules. If a helper knows about
the intake, cups, pins, or match strategy, it probably belongs somewhere else.

- `logger.hpp`: brain, terminal, and controller-friendly logging helpers.
- `math_utils.hpp`: small math helpers.
- `timer.hpp`: simple elapsed-time helper.

This folder should stay small. If a helper is only used once, leave it where it
is until it earns a spot here.
