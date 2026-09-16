# 构建与运行依赖

本仓库只跟踪源码、配置、构建脚本和文档。模型权重、SDK共享库、固件镜像、录像和运行日志不进入当前 Git 文件树。

## 源码依赖

`librm` 为 Git 子模块，固定提交 `52ec0174ee394d1fad06254b8947e8d60ba591d3`，含 etl 与 kalman 子模块。
使用 `git clone --recurse-submodules` 或 `git submodule update --init --recursive` 获取。其许可证随上游源码保留。
系统依赖沿用 ROS 2 Humble、OpenVINO 2024.6、OpenCV、Eigen、Ceres、fmt、spdlog、yaml-cpp。
固件需要独立安装 GNU Arm 工具链和 STM32CubeF4 SDK，见 `firmware/README.md`。

## 本机二进制依赖

相机 SDK 的头文件在 `io/`，共享库由厂商 SDK 提供。当前构建使用：

- `io/hikrobot/lib/amd64/libMvCameraControl.so`（ARM 为 `arm64`）
- `io/mindvision/lib/amd64/libMVSDK.so`（ARM 为 `arm64`）

当前正式模型文件为 `assets/yolo26.xml`、`assets/yolo26.bin`、`assets/tiny_resnet.onnx`。
这些文件仍保留在现有机器，只取消 Git 跟踪。当前运行资源的路径和哈希见 `artifacts.json`。
模型训练源码和可复现训练数据目前未提供，所以仅凭本仓库不能从头训练出相同权重；不得声称整个系统已完全可复现构建。

## 新克隆的资源准备

旧 Git 历史和本地历史备份已删除，源码仓库不再提供二进制资源恢复入口。
相机共享库需从对应厂商 SDK 安装；模型需由当前资源持有人提供，按 `artifacts.json` 中的路径放置，并核对 SHA-256。
运行 `python3 scripts/check_runtime_assets.py` 校验当前模型和 SDK。该脚本只读取文件，不联网、不恢复旧文件、不启动程序。
现有小电脑与 Windows 工作区的当前运行资源原地保留。仅克隆源码并不能获得模型、SDK 或直接运行程序。
