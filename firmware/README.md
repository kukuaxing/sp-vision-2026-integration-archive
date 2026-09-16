# 下位机固件

| 目录 | 硬件 | 当前来源 |
|---|---|---|
| `sentry/` | 新哨兵 STM32F405 | 9月14日 SentryUC 协议V3及 P 轴启动保持修复 |
| `infantry/` | 原步兵 STM32F405 | 6500 rpm、供弹门控与遥测版本 |

两者是不同硬件工程，不是同一程序的历史副本。每个目录只维护一套源码，不能混刷。
仓库只保存固件源码。HEX/BIN 等编译产物由本地构建生成，不进入 Git；现有机器上的镜像保留为忽略文件。本次没有烧录。
哨兵协议见 [PROTOCOL_V3.md](sentry/PROTOCOL_V3.md)，历史烧录证据见 [FLASH_REPORT.md](sentry/FLASH_REPORT.md)。报告中的备份路径是当时证据位置，不是当前开发入口。
哨兵构建需指定现有 STM32CubeF4 SDK 的 `CUBE_ROOT` 和 GNU Arm 工具链 `ARM_TOOLCHAIN_ROOT`，不要把 SDK 复制成新的工程版本。
