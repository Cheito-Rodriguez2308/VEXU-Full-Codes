# Style Notes

This code should survive a rushed tournament day. Prefer boring, readable code
over clever patterns.

## Naming

- Classes use `PascalCase`.
- Functions and variables use `camelCase`.
- Constants live in `config/constants.hpp` and use descriptive names.
- Enum values describe robot states, not button names.

## Comments

Write comments that explain why a choice exists or how to tune it. Avoid comments
that simply repeat the code.

Good:

```cpp
// Motion chaining trades perfect settle accuracy for smoother route timing.
```

Less useful:

```cpp
// Set motor voltage.
```

## Dependency Direction

```text
config -> no robot behavior
subsystems -> hardware behavior
control -> driver requests
auton -> route actions
tuning -> repeatable tests
core -> owns and wires modules
```

Subsystems should not include controller logic. Driver control and autonomous
should request subsystem states instead of directly controlling motors.

## Placeholder Values

Every unmeasured robot-specific value must include `TODO`. Replace
one group at a time and record the test result in `docs/`.
