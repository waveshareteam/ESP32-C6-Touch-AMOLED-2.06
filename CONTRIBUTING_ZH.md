# 贡献指南

[English](CONTRIBUTING.md)

感谢你改进 ESP32-C6-Touch-AMOLED-2.06 示例。

## 提交 Pull Request 之前

1. 将一方 ESP-IDF 应用保存在 `examples/esp-idf/<project>`。
2. 将一方 Arduino 草图保存在 `examples/arduino/<sketch>`，并使用同名
   `.ino` 主文件。
3. 可复用依赖优先使用 ESP-IDF 托管组件；新增本地组件时，请在
   [组件策略](docs/components_ZH.md)中说明原因。
4. 使用[硬件引脚审计](docs/hardware_ZH.md)中的引脚表，并根据原理图核验
   面向硬件的修改。
5. 不要提交构建目录、生成的 `sdkconfig`、`managed_components` 或发布 ZIP。
6. 不要使用源码构建结果替换工厂镜像。

## 框架兼容性

ESP-IDF 修改必须与 CI 中的 v5.5 和 v6.0 矩阵兼容。Arduino 修改必须使用
[Arduino 示例说明](examples/arduino/README_ZH.md)记录的 core 和 FQBN 编译。

新增示例时，请更新对应 README，并确保发现规则会选择该示例，同时不会误选
随仓库提供的库内示例。

## Pull Request 说明

请包含：

- 受影响的示例和硬件部分；
- 组件版本变化及其上游来源；
- 用于验证的 CI 任务；
- 已完成的实体板测试，或仍需进行的硬件验证；
- 行为或分区变化时的迁移与烧录说明。

## CI 失败

阅读完整的失败任务，从第一个真实的依赖、编译器、链接器或打包错误着手。
最终汇总行通常只是后续结果。共享 BSP、库、工作流或打包变更必须通过完整的
受影响矩阵。

## 提交与分支规范

使用清晰、符合仓库习惯的分支名和聚焦的提交信息。不要在提交、Pull Request、
Issue、发布说明或公开粘贴的日志中包含本地文件系统路径、用户名、网络位置或
特定环境的工具路径。

## 许可证

提交贡献即表示你同意：除非文件明确声明其他兼容许可证，你的修改按本仓库的
Apache License 2.0 提供。请保留随仓库提供的第三方库声明。
