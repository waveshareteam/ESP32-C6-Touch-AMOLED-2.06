<div align="center">
  <h1>ESP32-C6-Touch-AMOLED-2.06</h1>
  <p><strong>搭载 2.06 英寸 410 × 502 QSPI AMOLED 触摸屏的 ESP32-C6 开发板</strong></p>
  <p><a href="README.md">English</a></p>
  <p><img src="https://www.waveshare.com/media/catalog/product/cache/1/image/560x560/9df78eab33525d08d6e5fb8d27136e95/e/s/esp32-c6-touch-amoled-2.06-1.jpg" alt="ESP32-C6-Touch-AMOLED-2.06" width="420"></p>
  <p>
    <a href="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/esp-idf-examples.yml"><img alt="ESP-IDF Examples" src="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/esp-idf-examples.yml/badge.svg"></a>
    <a href="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/arduino-examples.yml"><img alt="Arduino Examples" src="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/arduino-examples.yml/badge.svg"></a>
    <a href="LICENSE"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-C6-Touch-AMOLED-2.06"></a>
  </p>
  <p>
    <a href="https://www.waveshare.com/esp32-C6-touch-amoled-2.06.htm">🌐 产品页面</a> ·
    <a href="https://docs.waveshare.com/ESP32-C6-Touch-AMOLED-2.06">📚 产品文档</a> ·
    <a href="examples/esp-idf/README_ZH.md">🧩 ESP-IDF 示例</a> ·
    <a href="examples/arduino/README_ZH.md">🔧 Arduino 示例</a> ·
    <a href="docs/firmware_ZH.md">📦 固件制品</a>
  </p>
</div>

---

## ✨ 概览

本仓库为 Waveshare ESP32-C6-Touch-AMOLED-2.06 提供示例软件、由源码构建的
固件包、工厂恢复固件和硬件参考资料。

本开发板在紧凑的手表式平台中集成 ESP32-C6、高分辨率 AMOLED 显示屏、
电容触摸、运动传感器、实时时钟、电源管理和音频接口。

## 🖥️ 硬件概览

| 功能 | 器件 / 接口 |
| --- | --- |
| MCU | ESP32-C6，最高 160 MHz 的 32 位 RISC-V 处理器 |
| 无线连接 | 2.4 GHz Wi-Fi 6、Bluetooth 5、Zigbee 3.0 和 Thread |
| 存储 | 16 MB 外置 Flash |
| 显示 | 2.06 英寸 410 × 502 QSPI AMOLED，CO5300 控制器 |
| 触摸 | FT3168 电容触摸控制器，I²C 接口 |
| 电源管理 | AXP2101，支持电池充电 |
| 运动传感器 | QMI8658 六轴 IMU |
| 实时时钟 | PCF85063 |
| 音频 | 板载音频编解码器、麦克风输入和扬声器输出 |
| 板级支持 | 托管组件：[`waveshare/esp32_c6_touch_amoled_2_06`](https://components.espressif.com/components/waveshare/esp32_c6_touch_amoled_2_06) |
| 硬件文件 | [原理图](Schematic/ESP32-C6-Touch-AMOLED-2.06-Schematic-V1.0.pdf)和[已核验引脚表](docs/hardware_ZH.md) |

## 📦 固件制品

每次源码构建成功后，GitHub Actions 都会生成可直接下载的 ZIP。进入对应的
ESP-IDF 或 Arduino 工作流运行，在 **Artifacts** 区域下载与示例及框架版本
匹配的压缩包。

每个压缩包都包含应用程序及配套二进制、准确的烧录偏移、SHA-256 校验值、
元数据清单和烧录辅助脚本。解压压缩包并通过
`python -m pip install esptool` 安装 esptool 后，运行生成的辅助脚本之一：

```sh
./flash.sh --port PORT
```

```bat
flash.bat --port PORT
```

> [!NOTE]
> 辅助脚本保留框架生成的偏移。不要使用其他示例或工厂镜像的偏移替换它们。

[firmware](firmware/) 下提交的镜像是独立的工厂恢复制品，不由 CI 重建。
详情见[固件制品与烧录](docs/firmware_ZH.md)。

## 🧪 示例

### ESP-IDF

| 示例 | 重点 |
| --- | --- |
| [01_AXP2101](examples/esp-idf/01_AXP2101/) | 电源管理与电池遥测 |
| [02_lvgl_demo_v9](examples/esp-idf/02_lvgl_demo_v9/) | 显示、触摸与 LVGL 9 演示 |
| [03_esp-brookesia](examples/esp-idf/03_esp-brookesia/) | ESP-Brookesia 手表式用户界面 |
| [04_qmi8658](examples/esp-idf/04_qmi8658/) | QMI8658 运动数据与 LVGL 用户界面 |
| [05_Spec_Analyzer](examples/esp-idf/05_Spec_Analyzer/) | 音频采集、DSP 与频谱可视化 |

### Arduino

| 示例 | 重点 |
| --- | --- |
| [01_HelloWorld](examples/arduino/01_HelloWorld/) | 显示屏启动 |
| [02_GFX_AsciiTable](examples/arduino/02_GFX_AsciiTable/) | GFX 文本与字符渲染 |
| [03_LVGL_PCF85063_simpleTime](examples/arduino/03_LVGL_PCF85063_simpleTime/) | 使用 LVGL 显示 RTC 时间 |
| [04_LVGL_QMI8658_ui](examples/arduino/04_LVGL_QMI8658_ui/) | 使用 LVGL 显示 IMU 数据 |
| [05_LVGL_AXP2101_ADC_Data](examples/arduino/05_LVGL_AXP2101_ADC_Data/) | 使用 LVGL 显示电源遥测 |
| [06_LVGL_Arduino_v9](examples/arduino/06_LVGL_Arduino_v9/) | LVGL 9 功能演示 |

随仓库提供的 Arduino 库位于
[`examples/arduino/libraries`](examples/arduino/libraries/)。这些库自带的上游
示例不会进入产品 CI 矩阵。

## 🛠️ 支持的工具链

| 开发框架 | 版本 | 固件构建数 |
| --- | --- | ---: |
| ESP-IDF | `v5.5.5` | 5 |
| ESP-IDF | `v6.0.2` | 5 |
| Arduino-ESP32 | `3.3.11` | 6 |

独立的 [ESP-IDF](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/esp-idf-examples.yml)
与 [Arduino](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/actions/workflows/arduino-examples.yml)
工作流各自运行一个始终可见的发现任务，并按需运行最多 16 个固件构建任务。
纯文档变更会跳过固件矩阵，但仍保留轻量策略和路由检查。每次成功构建都会
打包为可烧录制品。矩阵和手动触发方式见[持续集成](docs/ci_ZH.md)。

工具链版本和开发板选项的机器可读来源是
[`config/toolchains.json`](config/toolchains.json)。

## 🗂️ 仓库结构

| 路径 | 用途 |
| --- | --- |
| [`examples/esp-idf/`](examples/esp-idf/) | 一方 ESP-IDF 应用 |
| [`examples/arduino/`](examples/arduino/) | 一方 Arduino 草图和随仓库提供的库 |
| [`firmware/`](firmware/) | 工厂烧录与恢复镜像 |
| [`releases/`](releases/) | 固件打包工具 |
| [`Schematic/`](Schematic/) | 公开原理图文件 |
| [`config/`](config/) | CI 工具链版本和开发板选项 |
| [`docs/`](docs/) | CI、组件、硬件和固件说明 |

规范示例路径与框架版本无关。支持的 CI 基线变化时，应更新
`config/toolchains.json`，而不是创建带版本后缀的示例根目录。

## 📚 文档

- [产品文档](https://docs.waveshare.com/ESP32-C6-Touch-AMOLED-2.06)
- [硬件引脚审计](docs/hardware_ZH.md)
- [持续集成](docs/ci_ZH.md)
- [托管组件](docs/components_ZH.md)
- [固件制品与烧录](docs/firmware_ZH.md)
- [发布打包工具](releases/README_ZH.md)

## 🤝 支持与贡献

欢迎提交贡献和可复现的问题报告。请提供示例路径、框架版本、复现步骤、
预期行为、实际行为以及相关串口日志。

- [贡献指南](CONTRIBUTING_ZH.md)
- [支持](SUPPORT_ZH.md)
- [安全策略](SECURITY_ZH.md)
- [提交 Issue](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-2.06/issues/new/choose)

## 📄 许可证

本仓库采用 Apache License 2.0，详见 [LICENSE](LICENSE)。
