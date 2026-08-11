# 发布打包

[English](README.md)

`package_firmware.py` 把成功的 CI 构建输出转换为自包含 ZIP；它本身不编译固件。

对 ESP-IDF，它读取项目生成的 `build/flasher_args.json` 并保留框架给出的
烧录偏移。对 Arduino，它优先使用导出的 merged 二进制；必要时回退到标准
ESP32-C6 bootloader、分区和应用偏移。

打包器会拒绝逃出所选构建目录的 ESP-IDF 路径、存在歧义的 Arduino merged
镜像，以及在 CI 中缺少完整 Git 提交 SHA 的清单。

每个包包含：

- `bin/` 下的所有相关二进制；
- 带源码和框架元数据、偏移、大小与 SHA-256 的 `manifest.json`；
- `flash.sh` 和 `flash.bat` 辅助脚本；
- 适用时位于 `metadata/` 下的 ESP-IDF 原始 `flasher_args.json`。

工作流在构建成功后调用脚本：

```sh
python releases/package_firmware.py esp-idf \
  --project examples/esp-idf/01_AXP2101 \
  --build-dir examples/esp-idf/01_AXP2101/build \
  --framework-version v5.5.5 \
  --output-dir release-artifacts
```

生成的压缩包上传为 GitHub Actions 制品，不提交到仓库。
