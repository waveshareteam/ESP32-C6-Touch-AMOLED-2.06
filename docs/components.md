# Component policy

[简体中文](components_ZH.md)

Prefer the Espressif Component Registry for reusable ESP-IDF dependencies. Keep a
component local only when it contains project-specific integration or when a safe
registry replacement has not been established.

## Managed ESP-IDF dependencies

| Component | Constraint | Purpose |
| --- | --- | --- |
| `waveshare/esp32_c6_touch_amoled_2_06` | `^2.0.0` | Published board BSP |
| `waveshare/qmi8658` | `^2.0.0` | Board IMU driver |
| `cube32esp/xpowerslib` | `^0.3.3` | AXP2101 power-management API |
| `lvgl/lvgl` | `9.5.0` | GUI framework used by the examples |
| `espressif/esp-dsp` | `^1.8.2` | Spectrum-analyzer DSP routines |
| `espressif/esp-boost` | `^0.6.0` | C++ support used by the Brookesia snapshot |
| `chmorgan/esp-audio-player` | `^1.1.0` | Audio playback helper |
| `chmorgan/esp-file-iterator` | `^1.0.0` | File iteration for audio assets |

The AXP2101 application uses the managed XPowersLib component instead of carrying a
second copy of the same reusable source. Component resolution and API compatibility
are validated by both ESP-IDF CI versions.

## Intentionally local ESP-IDF code

### Brookesia snapshot

`examples/esp-idf/03_esp-brookesia/components` contains the Brookesia core plus the
board application's SquareLine UI integration. It is not treated as a generic
project-wide component. Updating it should be a dedicated change that compares the
local snapshot with its upstream source and exercises the entire Brookesia app on
hardware.

The dependency constraints inside this snapshot are retained where they express the
snapshot's own compatibility contract. Top-level board and LVGL dependencies are
kept explicit.

### Spectrum-analyzer audio extension

`examples/esp-idf/05_Spec_Analyzer/components/bsp_extra` binds the published board
BSP to application-specific audio and file-iteration behavior. It remains local
because moving it into the shared board BSP would change upstream board-component
scope.

The published board BSP owns the shared `BSP_I2S_NUM` Kconfig symbol. The local
extension therefore does not redefine that symbol. Its CMake dependency list selects
the split GPIO, I2C, I2S, and LEDC driver components on ESP-IDF 5.5 and newer while
retaining the legacy aggregate dependency for the older manifest-supported line.

## Arduino libraries

Arduino examples compile against `examples/arduino/libraries`. This is intentional:
the repository ships board-tested copies, and mixing them silently with globally
installed versions would make examples less reproducible. CI passes this directory
explicitly to Arduino CLI and does not compile the libraries' nested example trees.

The bundled Arduino_GFX 1.6.0 copy includes the upstream compatibility guard that
skips an unused legacy SPI clock-divider call on Arduino-ESP32 3.3.6 and newer. This
keeps the shipped library source compatible with the repository's 3.3.8 baseline.

## Update process

For a managed-component update:

1. Confirm the latest stable registry version and its ESP-IDF requirements.
2. Update the narrow manifest constraint.
3. Run all applications affected by the component on ESP-IDF v5.5 and v6.0 CI.
4. Inspect the first real CI error if dependency resolution or compilation fails.
5. Perform board validation when the component controls hardware behavior.

Do not replace a local component solely because a similarly named registry package
exists; compare API, board integration, local modifications, and licensing first.
