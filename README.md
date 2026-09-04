# 2026 视觉—下位机联调归档

本私有仓库归档截至 2026-09-04 的三部分成果：STM32F405 XUC V2 下位机固件、TJ-advanced 上位机通信/自瞄改动，以及格式塔系统中最终 v4q 仿真结果。

继续实车工作的 AI 或开发者请先完整阅读 [`HANDOFF_20260903.md`](HANDOFF_20260903.md)。它记录了当前 Flash 版本、安全边界、实车证据和下一轮严格执行顺序。

## 当前可确认状态

- 下位机当前实车已烧录 **Phase 2D.5 双轴版**，bin SHA-256 `7708D30EE510AB4807379175C4314B1D413BD3F96CB01A3F93A92B07937A1F21`；编译、烧录、回读和安全心跳均通过。
- Yaw、Pitch 单轴与双轴“∞”轨迹联调已实车通过；开火测试尚未开始。
- 上位机 XUC V2 基础改动已在 `kukuaxing/TJ-advanced` 的 `feat/xuc-board-communication-20260901` 分支，提交为 `ed41007501b34ab2e13a125d7d2820a180829a33`；2026-09-04 实车现场改动尚未合并回 TJ-advanced，以本仓库快照为准。
- 格式塔仿真最终候选为 **v4q**：正反转合计 10/10 个有效回合，300 发、228 次伤害命中，汇总命中率 76.0%。

## 目录

- `firmware/stm32f405-xuc-v2/`：下位机源码快照、Git bundle、历史产物和已烧录双轴 D5 产物/回读。
- `upper-computer/TJ-advanced-xuc-v2/`：上位机现场改动文件、最终配置与双轴实车日志。
- `simulation/gestalt-drive-aim-v4q/`：最终报告、逐轮结果、配置、源码及 10 个 v4q 正反转有效日志。
- `SHA256SUMS.txt`：本仓库除该文件和 `.git` 外所有归档文件的 SHA-256。

## 安全边界

双轴跟随已通过，但开火仍是独立高风险阶段。实车继续测试必须确保云台与射线无人、底盘架空、遥控可立即退出 AUTO，并从禁射配置起步。

## 恢复

固件源码可直接使用 `source/`，也可以从 `history/RM_Infantry-Robot-xuc-v2.bundle` 恢复原始 Git 历史。上位机完整工程继续以私有仓库 `kukuaxing/TJ-advanced` 为准；本仓库 `changed-files/` 同时包含已提交基础快照和 2026-09-04 现场未合并快照。
