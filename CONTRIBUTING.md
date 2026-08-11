# Contributing

[简体中文](CONTRIBUTING_ZH.md)

Thank you for improving the ESP32-C6-Touch-AMOLED-2.06 examples.

## Before opening a pull request

1. Keep first-party ESP-IDF applications under `examples/esp-idf/<project>`.
2. Keep first-party Arduino sketches under `examples/arduino/<sketch>` with a
   same-named `.ino` file.
3. Prefer managed ESP-IDF components for reusable dependencies; explain any new
   local component in `docs/components.md`.
4. Use the pin map in `docs/hardware.md` and verify hardware-facing changes against
   the schematic.
5. Do not commit build directories, generated `sdkconfig`, managed components, or
   release ZIPs.
6. Do not replace the factory image with a source build.

## Framework compatibility

ESP-IDF changes must remain compatible with the v5.5 and v6.0 CI matrix. Arduino
changes must compile with the core and FQBN documented in
`examples/arduino/README.md`.

When adding an example, update the relevant README and ensure the discovery rules
select it without also selecting bundled library examples.

## Pull request description

Include:

- the affected examples and hardware surfaces;
- component version changes and their upstream source;
- the CI jobs used for validation;
- physical board tests performed, or a clear statement that hardware validation is
  still required;
- migration or flashing notes when behavior or partitions change.

## CI failures

Read the complete failing job and act on the first real dependency, compiler, linker,
or packaging error. The final summary line is often only a consequence. Shared BSP,
library, workflow, or packaging changes must pass the complete affected matrix.

## Commit and branch hygiene

Use descriptive, repository-style branch names and focused commit messages. Do not
include local filesystem paths, usernames, network locations, or environment-specific
tool paths in commits, pull requests, issues, release notes, or logs pasted publicly.

## Licensing

By contributing, you agree that your changes are provided under the repository's
Apache License 2.0 unless a file clearly states another compatible license. Preserve
third-party notices in bundled libraries.
