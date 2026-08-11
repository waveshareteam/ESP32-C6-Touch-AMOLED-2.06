# Arduino examples

[简体中文](README_ZH.md)

The six direct child sketch directories are the product examples. The `libraries/`
directory contains the library versions shipped with the board; examples nested
inside those libraries are upstream reference material and are not part of the CI
matrix.

## CI board configuration

CI uses Arduino-ESP32 3.3.11 and this FQBN:

```text
esp32:esp32:esp32c6:CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
```

This selects the ESP32-C6 target, USB CDC on boot, 16 MB flash, and a 3 MB
application partition with FATFS space.

## Bundled libraries

Compile with `examples/arduino/libraries` as an additional library root. The board
pin definitions are registered as the `ESP32-C6-Touch-AMOLED-2.06 Board Config`
library in `libraries/Mylibrary` for compatibility with the existing sketches.

LVGL's configuration file is stored at `libraries/lv_conf.h`. Before compiling an
LVGL sketch with Arduino CLI, copy it into the sketch directory:

```sh
cp examples/arduino/libraries/lv_conf.h examples/arduino/06_LVGL_Arduino_v9/lv_conf.h
arduino-cli compile \
  --fqbn "esp32:esp32:esp32c6:CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB" \
  --libraries examples/arduino/libraries \
  examples/arduino/06_LVGL_Arduino_v9
```

The CI copy is temporary and is not committed.

## First-party sketches

- `01_HelloWorld`
- `02_GFX_AsciiTable`
- `03_LVGL_PCF85063_simpleTime`
- `04_LVGL_QMI8658_ui`
- `05_LVGL_AXP2101_ADC_Data`
- `06_LVGL_Arduino_v9`

Changing a sketch builds only that sketch. Changing a bundled library, the Arduino
workflow, shared routing logic, or firmware packager builds all six sketches.
Markdown-only changes select no firmware builds. Missing diff data fails closed
instead of selecting a silent fallback matrix.

Successful builds upload directly flashable ZIP artifacts. See
[the firmware guide](../../docs/firmware.md).
