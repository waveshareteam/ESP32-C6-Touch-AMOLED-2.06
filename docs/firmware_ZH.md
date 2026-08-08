# 固件制品与烧录

[English](firmware.md)

本仓库包含两类彼此独立的固件来源：

1. `firmware/` 保存由工厂提供的镜像，用于恢复或参考。
2. GitHub Actions 制品在当前源码成功完成 ESP-IDF 或 Arduino 构建后生成。

工厂镜像不会被重新发现、重建，也不会作为源码构建制品上传。

## CI 制品内容

每个源码构建 ZIP 包含：

```text
bin/                    应用程序及配套二进制
manifest.json           开发板、框架、源码修订、偏移、大小和哈希
flash.sh                POSIX shell 烧录辅助脚本
flash.bat               Windows 命令提示符烧录辅助脚本
metadata/               可用时保存框架元数据（ESP-IDF 包）
```

清单记录 ESP32-C6 目标、框架版本、仓库相对源码项目、Git 提交 SHA、生成时间、
烧录命令、文件大小和 SHA-256。

## 前置条件

- 支持数据传输的 USB 线和可见串口
- Python 3
- Espressif `esptool` 软件包

请在隔离的 Python 环境中安装烧录工具：

```sh
python -m pip install esptool
```

## 烧录 CI 包

1. 下载所需示例和框架版本对应的制品。
2. 完整解压 ZIP。
3. 如果自动复位未进入下载模式，手动让开发板进入下载模式。
4. 在解压目录运行辅助脚本。

Linux 或 macOS：

```sh
./flash.sh --port PORT
```

Windows 命令提示符：

```bat
flash.bat --port PORT
```

生成的辅助脚本包含准确的二进制偏移。如需串口参数，可将其添加到
`manifest.json` 中展示的 `python -m esptool` 命令，或在运行前编辑辅助脚本。
预先擦除 Flash 是可选且具有破坏性的操作；执行前应备份设备专用数据。

## 工厂镜像

使用工厂镜像前请阅读 [firmware 说明](../firmware/README_ZH.md)。工厂镜像可能
包含与源码示例不同的应用、分区表或设备设置。不要混用 CI 包偏移和工厂二进制。

## 打包实现

`releases/package_firmware.py` 读取 ESP-IDF `flasher_args.json` 或 Arduino CLI
导出的二进制。只有源码构建成功后，它才创建按项目和框架版本分开的 ZIP。
单元测试使用合成二进制，不构建固件。

打包器会拒绝解析到所选构建目录之外的 ESP-IDF 文件引用、存在歧义的 Arduino
merged 镜像，以及在 CI 中缺少完整 Git 提交 SHA 的清单。

生成的 ZIP 应保存在 CI 制品或有意创建的 GitHub Release 中，不提交到源码树。
