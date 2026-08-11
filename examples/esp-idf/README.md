# ESP-IDF examples

[简体中文](README_ZH.md)

These first-party applications target `esp32c6` and are built in CI with ESP-IDF
v5.5.5 and v6.0.2.

| Directory | Main managed dependencies |
| --- | --- |
| `01_AXP2101` | `cube32esp/xpowerslib ^0.3.3` |
| `02_lvgl_demo_v9` | Waveshare board BSP `^2.0.0`, LVGL `9.5.0` |
| `03_esp-brookesia` | Waveshare board BSP `^2.0.0`, LVGL `9.5.0` |
| `04_qmi8658` | Waveshare board BSP `^2.0.0`, QMI8658 `^2.0.0`, LVGL `9.5.0` |
| `05_Spec_Analyzer` | ESP-DSP `^1.8.2`, LVGL `9.5.0`, project audio extension |

## Build one project

Install a supported ESP-IDF release, then run:

```sh
cd examples/esp-idf/02_lvgl_demo_v9
idf.py set-target esp32c6
idf.py build
```

The Component Manager downloads registry dependencies into `managed_components/`.
That generated directory and the `build/` directory are not committed.

## Project-local components

`03_esp-brookesia/components` is a board-integrated Brookesia snapshot with local UI
and compatibility changes. It is kept in the application because replacing it with
an unrelated registry release would change the application surface.

`05_Spec_Analyzer/components/bsp_extra` is application-specific audio integration
for the board. It is not a duplicate of the published Waveshare board BSP.

## CI selection

The discovery script selects only direct child projects of this directory that have
both `CMakeLists.txt` and `main/`. Nested component test applications are therefore
not mistaken for product examples. The shared changed-file router selects one
affected project when possible, skips Markdown-only and `firmware/` changes, and
fails closed on incomplete diff data. A workflow, packaging, discovery, or routing
change runs all five applications on both supported ESP-IDF versions.

CI produces one flashable ZIP per project and framework version. See
[the firmware guide](../../docs/firmware.md).
