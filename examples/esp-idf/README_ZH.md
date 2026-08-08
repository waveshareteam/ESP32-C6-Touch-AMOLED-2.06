# ESP-IDF 示例

[English](README.md)

这些一方应用以 `esp32c6` 为目标，并在 CI 中使用 ESP-IDF v5.5.5 和
v6.0.2 构建。

| 目录 | 主要托管依赖 |
| --- | --- |
| `01_AXP2101` | `cube32esp/xpowerslib ^0.3.3` |
| `02_lvgl_demo_v9` | Waveshare 开发板 BSP `^2.0.0`、LVGL `9.5.0` |
| `03_esp-brookesia` | Waveshare 开发板 BSP `^2.0.0`、LVGL `9.5.0` |
| `04_qmi8658` | Waveshare 开发板 BSP `^2.0.0`、QMI8658 `^2.0.0`、LVGL `9.5.0` |
| `05_Spec_Analyzer` | ESP-DSP `^1.8.2`、LVGL `9.5.0`、项目音频扩展 |

## 构建一个项目

安装受支持的 ESP-IDF 版本后运行：

```sh
cd examples/esp-idf/02_lvgl_demo_v9
idf.py set-target esp32c6
idf.py build
```

Component Manager 会把 registry 依赖下载到 `managed_components/`。该生成目录
和 `build/` 目录都不提交。

## 项目本地组件

`03_esp-brookesia/components` 是集成开发板的 Brookesia 快照，包含本地 UI
和兼容性修改。它保留在应用中，因为使用无关 registry 版本替换会改变应用表面。

`05_Spec_Analyzer/components/bsp_extra` 是本开发板的应用专用音频集成，不是
已发布 Waveshare 开发板 BSP 的重复副本。

## CI 选择

发现脚本仅选择本目录中同时包含 `CMakeLists.txt` 和 `main/` 的直接子项目，
所以嵌套组件测试应用不会被误当成产品示例。共享变更路由器会尽量只选择一个
受影响项目，跳过纯 Markdown 和 `firmware/` 变更，并在差异数据不完整时按
失败关闭。工作流、打包、发现或路由变更会在两个受支持 ESP-IDF 版本上运行
全部 5 个应用。

CI 为每个项目和框架版本生成一个可烧录 ZIP，详见
[固件指南](../../docs/firmware_ZH.md)。
