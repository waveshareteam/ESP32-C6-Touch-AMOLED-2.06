# 硬件引脚审计

[English](hardware.md)

面向开发板的定义已经与仓库原理图和已发布的 Waveshare ESP-IDF 开发板组件
交叉核对。本仓库的硬件事实来源仍是
[`ESP32-C6-Touch-AMOLED-2.06-Schematic-V1.0.pdf`](../Schematic/ESP32-C6-Touch-AMOLED-2.06-Schematic-V1.0.pdf)
所代表的开发板版本。

## 显示与触摸

| 功能 | ESP32-C6 GPIO |
| --- | ---: |
| AMOLED QSPI 时钟 | 0 |
| AMOLED QSPI 数据 0 | 1 |
| AMOLED QSPI 数据 1 | 2 |
| AMOLED QSPI 数据 2 | 3 |
| AMOLED QSPI 数据 3 | 4 |
| AMOLED 片选 | 5 |
| AMOLED 复位 | 11 |
| 触摸 I²C SDA | 8 |
| 触摸 I²C SCL | 7 |
| 触摸复位 | 10 |
| 触摸中断 | 15 |

屏幕分辨率为 410 × 502。Arduino 定义
`examples/arduino/libraries/Mylibrary/pin_config.h` 与此表一致。

## 共享 I²C 与运动传感器

| 功能 | ESP32-C6 GPIO |
| --- | ---: |
| 共享 I²C SDA | 8 |
| 共享 I²C SCL | 7 |
| QMI8658 中断 1 | 16 |
| QMI8658 中断 2 | 17 |

触摸控制器、AXP2101 PMU、QMI8658 IMU 和 RTC 共享开发板 I²C 总线。
AXP2101 示例配置 GPIO 7/8，并把 PMU 中断视为未连接（`-1`），与当前应用
行为一致。

## 音频

| 功能 | ESP32-C6 GPIO | 相对于 ESP32-C6 的方向 |
| --- | ---: | --- |
| I²S MCLK | 19 | 输出 |
| I²S BCLK/SCLK | 20 | 输出 |
| 编解码器/ADC 数据（`ASDOUT`） | 21 | 输入 |
| I²S LRCK | 22 | 输出 |
| 编解码器 DAC 数据（`DSDIN`） | 23 | 输出 |
| 功放控制 | 6 | 输出 |

这些方向很重要：编解码器 `ASDOUT` 是进入 MCU 的数据，`DSDIN` 是 MCU
发出的播放数据。

## 配置清理

本开发板上的 ESP32-C6 没有外部 Octal PSRAM，其支持的 CPU 频率也不使用
遗留的 ESP32-S3 240 MHz 设置。因此应用默认配置设置
`CONFIG_IDF_TARGET="esp32c6"`，并省略继承而来的 S3、PSRAM 和 240 MHz 选项。

## 验证边界

本引脚审计证明原理图、BSP 与源码定义一致。CI 只验证编译。生产发布前仍需在
实体板上验证显示、触摸、IMU 方向、麦克风、扬声器、PMU 和充电行为。
