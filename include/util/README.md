# Utility Headers

Utilities are small helpers shared by several modules. If a helper knows about
the intake, cups, pins, or match strategy, it probably belongs somewhere else.

Files:

- `logger.hpp`: brain, terminal, and controller-friendly logging helpers.
- `math_utils.hpp`: small math helpers.
- `timer.hpp`: simple elapsed-time helper.

Keep utilities:

- Keep helpers generic.
- Avoid robot strategy here.
- Prefer clear names over clever shortcuts.

Safe edits:

- Log formatting.
- Extra motion samples during tuning.
- Small math helpers only when they remove duplication.

Common mistakes:

- Turning utilities into a dumping ground.
- Adding hardware-specific behavior here.
- Logging too much during driver control.
