# Headers

Headers describe what each part of the robot can do. They should stay easy to
read in GitHub because new programmers usually start here before jumping into
the matching `.cpp` file.

Keep in mind:

- Keep class interfaces small and readable.
- Avoid hardware values in subsystem headers.
- Prefer enums and named structs over raw numbers.
- Put behavior in `src/`, not in the header unless it is a tiny helper.

Headers may include PROS and LemLib types when the public interface truly needs
them. If the type is only used inside one `.cpp`, keep the include there.

Common edits:

- Add new subsystem interfaces when the robot gains a real mechanism.
- Update config types when new hardware needs structured settings.

Watch for:

- Putting final robot ports in subsystem headers.
- Letting driver control talk directly to motors.
- Adding a helper before there is repeated code to simplify.
