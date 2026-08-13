# CI policy

[简体中文](ci_ZH.md)

GitHub Actions is the build-validation authority for this repository. The workflows
build source; the committed factory image is not treated as a source-build target.

## Build matrices

| Workflow | First-party targets | Framework versions |
| --- | ---: | --- |
| `esp-idf-examples.yml` | 5 applications | ESP-IDF v5.5.5, v6.0.2 |
| `arduino-examples.yml` | 6 sketches | Arduino-ESP32 3.3.8 |

Both workflows target ESP32-C6. The ESP-IDF matrix provides compatibility coverage
for the latest stable 5.5 and 6.0 release lines. The Arduino workflow uses the
latest supported stable core baseline recorded in this repository.
Versions, target, Arduino board-manager URL, and FQBN are read from
`config/toolchains.json`. Changing that file selects the complete matrix for both
frameworks.

## Discovery rules

ESP-IDF discovery accepts only direct children of `examples/esp-idf` containing a
top-level `CMakeLists.txt` and `main/` directory. This excludes component tests and
library examples.

Arduino discovery accepts only direct children of `examples/arduino` where the
directory and primary `.ino` file have the same name. Everything under
`examples/arduino/libraries` is excluded from discovery.

One shared changed-file router receives the complete rename-aware diff on pull
requests and pushes:

- A changed first-party project or sketch selects that target.
- A shared Arduino library change selects all six sketches.
- A workflow, discovery-script, test, or firmware-packager change selects every
  target for the affected framework.
- Markdown at the repository root, beside an example, or inside a bundled library
  selects no firmware builds.
- Files below `firmware/` are reported separately and never enter the default
  example matrices.
- An unfamiliar non-document path conservatively selects both complete matrices
  and remains visible in the routing report.
- Missing, empty, or unreadable diff data fails the discovery job; it never becomes
  a silent full build or successful no-op.

`workflow_dispatch` accepts a directory name, repository-relative path, or `all`.

The always-visible `repository-policy.yml` workflow checks first-party bilingual
pairs, reciprocal and same-language navigation, local links and fragments, homepage
structure, and public-text hygiene. Its homepage and exemption contract is read from
`config/markdown-audit.json`. The two build workflows also remain visible on every
pull request, but their expensive matrix jobs run only when the router selects
source-build inputs.

## Build and package sequence

1. Checkout the source with sufficient history for rename-aware routing.
2. Run Python unit tests for discovery, routing, Markdown policy, and packaging.
3. Fail closed if the complete changed-file scope cannot be read.
4. Build every selected source target.
5. Package successful build output into a flashable ZIP.
6. Upload each ZIP as a separate workflow artifact for 14 days.

Packaging is deliberately downstream of compilation. A ZIP artifact therefore
means the corresponding source build succeeded.

## ESP-IDF 6 compatibility

Applications explicitly target `esp32c6` and use component manifests compatible
with both matrix versions. The source avoids relying on legacy driver dependency
leakage and includes the ESP-IDF 6 compatibility changes already exercised by the
matrix.

When CI fails, diagnose the first real compiler or dependency error rather than the
final summary line. Fix the cause in source or manifests and rerun the complete
affected matrix.

## What CI does not prove

CI verifies dependency resolution, compilation, and packaging. It does not provide
hardware-in-the-loop validation. Display timing, touch calibration, audio levels,
sensor orientation, battery behavior, and sleep/wake behavior still require a real
board test before a release is promoted.
