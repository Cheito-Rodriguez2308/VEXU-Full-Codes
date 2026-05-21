# Include Directory

## Purpose

Public headers for the team-owned robot architecture live here. Headers define
interfaces and data shapes; source files in `src/` contain behavior.

## Responsibilities

- Keep class interfaces small and readable.
- Avoid hardware values in subsystem headers.
- Prefer enums and named structs over raw numbers.

## Dependencies

Headers may include PROS and LemLib types when the interface truly needs them.
Avoid including heavy headers just for implementation details.

## Students Should Modify

- Add new subsystem interfaces when the robot gains a real mechanism.
- Update config types when new hardware needs structured settings.

## Common Mistakes

- Putting final robot ports in subsystem headers.
- Letting driver control talk directly to motors.
- Adding clever abstractions before there is repeated code to simplify.
