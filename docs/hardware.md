# Hardware pin audit

The board-facing definitions were cross-checked against the repository schematic
and the published Waveshare ESP-IDF board component. The source of truth for this
repository remains the board revision represented by
[`ESP32-C6-Touch-AMOLED-2.06-Schematic-V1.0.pdf`](../Schematic/ESP32-C6-Touch-AMOLED-2.06-Schematic-V1.0.pdf).

## Display and touch

| Function | ESP32-C6 GPIO |
| --- | ---: |
| AMOLED QSPI clock | 0 |
| AMOLED QSPI data 0 | 1 |
| AMOLED QSPI data 1 | 2 |
| AMOLED QSPI data 2 | 3 |
| AMOLED QSPI data 3 | 4 |
| AMOLED chip select | 5 |
| AMOLED reset | 11 |
| Touch I²C SDA | 8 |
| Touch I²C SCL | 7 |
| Touch reset | 10 |
| Touch interrupt | 15 |

The panel resolution is 410 × 502 pixels. The Arduino definitions in
`examples/arduino/libraries/Mylibrary/pin_config.h` match this table.

## Shared I²C and motion sensor

| Function | ESP32-C6 GPIO |
| --- | ---: |
| Shared I²C SDA | 8 |
| Shared I²C SCL | 7 |
| QMI8658 interrupt 1 | 16 |
| QMI8658 interrupt 2 | 17 |

The touch controller, AXP2101 PMU, QMI8658 IMU, and RTC share the board I²C bus.
The AXP2101 example configures GPIO 7/8 and treats the PMU interrupt as not
connected (`-1`), matching its current application behavior.

## Audio

| Function | ESP32-C6 GPIO | Direction at ESP32-C6 |
| --- | ---: | --- |
| I²S MCLK | 19 | Output |
| I²S BCLK/SCLK | 20 | Output |
| Codec/ADC data (`ASDOUT`) | 21 | Input |
| I²S LRCK | 22 | Output |
| Codec DAC data (`DSDIN`) | 23 | Output |
| Power-amplifier control | 6 | Output |

These directions are important: the codec's `ASDOUT` signal is data into the MCU,
while `DSDIN` is playback data from the MCU.

## Configuration cleanup

ESP32-C6 has no external octal PSRAM on this board and its supported CPU frequency
does not use the stale ESP32-S3 240 MHz setting. The application defaults therefore
set `CONFIG_IDF_TARGET="esp32c6"` and omit inherited S3/PSRAM/240 MHz options.

## Validation boundary

The pin audit establishes consistency among schematic, BSP, and source definitions.
CI verifies compilation only. Validate display output, touch, IMU orientation,
microphones, speaker, PMU behavior, and charging on the physical board before a
production release.
