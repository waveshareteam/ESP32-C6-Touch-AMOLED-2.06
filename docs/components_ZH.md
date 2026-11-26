# 组件策略

[English](components.md)

可复用 ESP-IDF 依赖优先使用 Espressif Component Registry。只有当组件包含
项目专用集成，或尚未确认安全的 registry 替代项时，才保留本地组件。

## 托管 ESP-IDF 依赖

| 组件 | 约束 | 用途 |
| --- | --- | --- |
| `waveshare/esp32_c6_touch_amoled_2_06` | `^2.0.0` | 已发布的开发板 BSP |
| `waveshare/qmi8658` | `^2.0.0` | 板载 IMU 驱动 |
| `cube32esp/xpowerslib` | `^0.3.3` | AXP2101 电源管理 API |
| `lvgl/lvgl` | `9.5.0` | 示例使用的 GUI 框架 |
| `espressif/esp-dsp` | `^1.8.2` | 频谱分析器 DSP 例程 |
| `espressif/esp-boost` | `^0.6.0` | Brookesia 快照使用的 C++ 支持 |
| `chmorgan/esp-audio-player` | `^1.1.0` | 音频播放辅助组件 |
| `chmorgan/esp-file-iterator` | `^1.0.0` | 音频资源文件迭代 |

AXP2101 应用使用托管的 XPowersLib，而不是携带第二份相同的可复用源码。
两个 ESP-IDF CI 版本共同验证组件解析和 API 兼容性。

## 有意保留的本地 ESP-IDF 代码

### Brookesia 快照

`examples/esp-idf/03_esp-brookesia/components` 包含 Brookesia core 和开发板
应用的 SquareLine UI 集成。它不是通用的项目级组件。更新该快照应作为独立
变更，对比本地源码与上游，并在硬件上验证完整 Brookesia 应用。

快照内部表达其自身兼容性契约的依赖约束会被保留；顶层开发板和 LVGL 依赖保持
显式声明。

### 频谱分析器音频扩展

`examples/esp-idf/05_Spec_Analyzer/components/bsp_extra` 将已发布的开发板
BSP 与应用专用音频及文件迭代行为连接起来。把它移入共享开发板 BSP 会改变
上游组件范围，因此继续保留在本项目中。

已发布 BSP 拥有共享的 `BSP_I2S_NUM` Kconfig 符号，本地扩展不会重复定义。
其 CMake 依赖在 ESP-IDF 5.5 及更高版本选择拆分后的 GPIO、I2C、I2S 和
LEDC 驱动组件，同时为清单仍支持的旧版本保留聚合驱动依赖。

## Arduino 库

Arduino 示例针对 `examples/arduino/libraries` 编译。仓库提供经过开发板验证的
库版本；静默混用全局安装版本会降低可复现性，因此 CI 显式传入该目录，也不会
编译库内嵌套的示例树。

随仓库提供的 Arduino_GFX 1.6.0 已包含上游兼容性保护：在 Arduino-ESP32
3.3.6 及更高版本跳过未使用的旧 SPI 时钟分频调用，从而与本仓库 3.3.8
基线保持兼容。

## 更新流程

更新托管组件时：

1. 确认最新稳定 registry 版本及其 ESP-IDF 要求。
2. 更新范围明确的清单约束。
3. 在 ESP-IDF v5.5 和 v6.0 CI 上运行受影响的全部应用。
4. 依赖解析或编译失败时检查第一个真实 CI 错误。
5. 控制硬件行为的组件还必须执行开发板验证。

不要仅因 registry 中存在同名组件就替换本地组件；应先比较 API、开发板集成、
本地修改和许可证。
