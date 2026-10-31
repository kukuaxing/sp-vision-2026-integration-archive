# 当前实车固件交付

2026-10-31正式版的实车工程来源为用户本机 `H:/rm/Sentinel_robot-main(1)/Sentinel_robot-main/STM32F405`。队友调整的PID、电机ID、任务计数及本轮AUTO待机保持修正都以该工程为准。

`STM32F405/` 本体继续保留仓库既有参考实现和回归测试。`current_vehicle.patch` 保存参考源码到实车源码的完整差异，`current_vehicle_manifest.json` 保存基准和实车源码的SHA-256，避免复制第二份完整工程或将历史0x82诊断误认成当前实车功能。

如需恢复实车源码，在干净、对应正式版提交的检出目录中运行：

```sh
git apply --check --directory=firmware/sentry firmware/sentry/current_vehicle.patch
git apply --directory=firmware/sentry firmware/sentry/current_vehicle.patch
```

此操作将现有参考源码变为实车版本，包括删除实车不存在的参考辅助头。随后核对清单中的current_source哈希；不能把参考固件的测试结果当作补丁应用后的测试结果。补丁默认不自动应用，也不自动烧录。

源文件按UTF-8/LF归一化计算清单哈希，原工程未被修改。用户现场采用VS+VisualGDB；如改用本目录CMake与已有HAL/CMSIS/FreeRTOS，需另行构建验收，工具链、BSP和链接脚本可能导致生成二进制不同。清单所列VS镜像哈希仅标识本机已有构建，最新镜像未在本次归档重新读板验证。

原始运行资源、构建镜像、个人IDE设置不上传；保留现有版权及许可范围。本次只归档，不修改独立实车工程、不烧录、不启动动作。
