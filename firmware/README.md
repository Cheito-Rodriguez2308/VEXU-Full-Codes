# Firmware

## Purpose

PROS, LemLib, LVGL, and linker artifacts live here. These files are part of the
PROS project template and are required for builds.

## Students Should Modify

Usually nothing. The only team-owned change is `hot-cold-asset.mk`, which filters
static assets so nested `static/paths/*.txt` LemLib paths build correctly.

## Common Mistakes

- Deleting template libraries.
- Editing linker scripts without a specific PROS toolchain reason.
- Adding non-path documentation files as robot binary assets.
