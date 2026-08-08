# Arduino 示例

[English](README.md)

6 个直接子目录中的草图是一方产品示例。`libraries/` 保存开发板随附的库版本；
这些库内嵌套的示例属于上游参考资料，不进入 CI 矩阵。

## CI 开发板配置

CI 使用 Arduino-ESP32 3.3.11 和以下 FQBN：

```text
esp32:esp32:esp32c6:CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
```

该配置选择 ESP32-C6 目标、启动时启用 USB CDC、16 MB Flash，以及带 FATFS
空间的 3 MB 应用分区。

## 随仓库提供的库

编译时把 `examples/arduino/libraries` 作为额外库根目录。开发板引脚定义注册为
`libraries/Mylibrary` 中的 `ESP32-C6-Touch-AMOLED-2.06 Board Config` 库，
以兼容现有草图。

LVGL 配置文件保存在 `libraries/lv_conf.h`。使用 Arduino CLI 编译 LVGL 草图
前，把它复制到草图目录：

```sh
cp examples/arduino/libraries/lv_conf.h examples/arduino/06_LVGL_Arduino_v9/lv_conf.h
arduino-cli compile \
  --fqbn "esp32:esp32:esp32c6:CDCOnBoot=cdc,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB" \
  --libraries examples/arduino/libraries \
  examples/arduino/06_LVGL_Arduino_v9
```

CI 中的复制是临时操作，不会提交。

## 一方草图

- `01_HelloWorld`
- `02_GFX_AsciiTable`
- `03_LVGL_PCF85063_simpleTime`
- `04_LVGL_QMI8658_ui`
- `05_LVGL_AXP2101_ADC_Data`
- `06_LVGL_Arduino_v9`

修改一个草图只构建该草图。修改随仓库提供的库、Arduino 工作流、共享路由逻辑
或固件打包器会构建全部 6 个草图。纯 Markdown 变更不选择固件构建；缺失的
差异数据会按失败关闭，而不是静默选择后备矩阵。

构建成功后会上传可直接烧录的 ZIP，详见
[固件指南](../../docs/firmware_ZH.md)。
