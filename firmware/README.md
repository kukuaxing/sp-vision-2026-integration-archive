# 下位机固件

| 目录 | 硬件 | 当前来源 |
|---|---|---|
| `sentry/` | 新哨兵 STM32F405 | SentryUC master `6a107ca` 保持目标 32685（上抬约 8°）；自动回归已验证，遥控已加入 |
| `infantry/` | 原步兵 STM32F405 | 6500 rpm、供弹门控与遥测版本 |

两者是不同硬件工程，不是同一程序的历史副本。每个目录只维护一套源码，不能混刷。
仓库只保存固件源码。HEX/BIN 等编译产物由本地构建生成于 `build/`，不进入 Git。
哨兵当前源码及构建适配见 [SOURCE.md](sentry/SOURCE.md)。原版不含先前本地协议 V3 与俯仰启动保持补丁。
烧录和读回校验已通过，当前 P 轴自动回归目标 32685 已验证，已加入云台遥控；实际摇杆运动及 Yaw 通信待现场验证；详见 [FLASH_REPORT.md](sentry/FLASH_REPORT.md)。
哨兵构建需指定现有 STM32CubeF4 SDK 的 `CUBE_ROOT` 和 GNU Arm 工具链 `ARM_TOOLCHAIN_ROOT`，不要把 SDK 复制成新的工程版本。
