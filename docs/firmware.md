# Firmware artifacts and flashing

[简体中文](firmware_ZH.md)

There are two distinct firmware sources in this repository.

1. `firmware/` contains a factory-provided image retained for recovery or reference.
2. GitHub Actions artifacts are generated from the current source after a successful
   ESP-IDF or Arduino build.

The factory image is never rediscovered, rebuilt, or uploaded as a source-build
artifact.

## CI artifact contents

Each source-build ZIP contains:

```text
bin/                    Application and supporting binaries
manifest.json           Board, framework, source revision, offsets, sizes, hashes
flash.sh                POSIX shell flashing helper
flash.bat               Windows command prompt flashing helper
metadata/               Framework metadata when available (ESP-IDF bundles)
```

The manifest records the ESP32-C6 target, framework version, repository-relative
source project, Git commit SHA, generation time, flash command, file sizes, and
SHA-256 hashes.

## Prerequisites

- A data-capable USB cable and a visible serial port
- Python 3
- Espressif's `esptool` package

Install the flashing tool in an isolated Python environment:

```sh
python -m pip install esptool
```

## Flash a CI bundle

On Windows, `Flash-CI-Firmware.cmd` at the repository root provides the guided
workflow used for sequential hardware checks. It resolves the local checkout's
exact HEAD, accepts only successful Actions runs for that revision, validates the
manifest identity and every binary's size and SHA-256, and then flashes the current
item. After checking the board, choose **Mark PASS and flash next** to record the
result and advance exactly one item. Progress resets automatically when HEAD changes.

Wait for both example workflows to finish successfully before launching the tool.
Normal mode also requires a completely clean working tree (no staged, unstaged, or
untracked files) and exactly one open, non-draft pull request for the checked-out
non-detached branch. That pull request's full head SHA must match local HEAD; the tool
checks the local tree before network access and stops before downloading or flashing
when the pull request identity does not match.

It auto-selects a port only when exactly one present USB serial device matches
Espressif VID 303A and PID 1001. In every other case, launch it with an explicit
port, for example:

```bat
Flash-CI-Firmware.cmd -Port COMx
```

The manual bundle flow remains available on every platform:

1. Download the artifact for the desired example and framework version.
2. Extract the ZIP completely.
3. Put the board into download mode if automatic reset does not do so.
4. Run the helper from the extracted directory.

On Linux or macOS:

```sh
./flash.sh --port PORT
```

On Windows Command Prompt:

```bat
flash.bat --port PORT
```

The generated helpers contain the exact binary offsets. If a port argument is
needed, either add it to the `python -m esptool` command shown in `manifest.json` or
edit the helper before running it. Erasing flash first is optional and destructive;
back up device-specific data before doing so.

## Factory image

See [firmware/README.md](../firmware/README.md) before using the factory image. A
factory image may contain a different application, partition table, or device setup
than the source examples. Do not combine offsets from a CI bundle with factory
binary files.

## Packaging implementation

`releases/package_firmware.py` consumes ESP-IDF `flasher_args.json` or Arduino CLI
exported binaries. It creates separate per-project, per-framework-version ZIPs only
after the source build succeeds. Unit tests use synthetic binaries and do not build
firmware.

The packager rejects ESP-IDF file references that resolve outside the selected build
directory, refuses ambiguous Arduino merged images, and requires a complete Git
commit SHA in CI-generated manifests.

Generated ZIP files belong in CI artifacts or an intentional GitHub release; they
are not committed to the source tree.
