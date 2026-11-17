# Factory firmware

[简体中文](README_ZH.md)

| File | Purpose |
| --- | --- |
| [ESP32-C6-Touch-AMOLED-2.06_FactoryOnly_20261029.bin](ESP32-C6-Touch-AMOLED-2.06_FactoryOnly_20261029.bin) | Factory recovery firmware, 2026-10-29. Flash at `0x0`. |
| [ESP32-C6-Touch-AMOLED-2.06-xiaozhi-251028.bin](ESP32-C6-Touch-AMOLED-2.06-xiaozhi-251028.bin) | Previously supplied XiaoZhi image. |

Before flashing it:

- Back up any device-specific data that must be preserved.
- Confirm the board model and hardware revision.
- Use the flashing offset and procedure from the product documentation associated
  with this image; do not infer offsets from source-build artifacts.
- Do not mix this binary with bootloader or partition files from another build.

GitHub Actions deliberately excludes this directory from ESP-IDF and Arduino source
discovery. Successful CI source builds create separate traceable ZIP artifacts with
their own manifests and flashing commands.

See [the firmware guide](../docs/firmware.md) for the distinction between factory and
source-build firmware.

## Updating a factory image

Treat a factory-binary replacement as a release operation. Record its origin,
expected hardware revision, flash procedure, checksum, and user-visible version in
the pull request. Never overwrite the existing image with an unrelated local build.
