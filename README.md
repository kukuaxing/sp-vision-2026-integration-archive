# 2026 视觉—下位机联调归档

本私有仓库归档 2026-09-03 时点的三部分成果：STM32F405 XUC V2 下位机固件、TJ-advanced 上位机通信/自瞄改动，以及格式塔系统中最终 v4q 仿真结果。

继续实车工作的 AI 或开发者请先完整阅读 [`HANDOFF_20260903.md`](HANDOFF_20260903.md)。它记录了当前 Flash 版本、安全边界、实车证据和下一轮严格执行顺序。

## 当前可确认状态

- 下位机当前实车烧录的是 **Phase 2D.4**，源码提交 `331f080c9dd0d213e513a5c0d412421aca2ebae8`；烧录与同长度回读校验已有日志留档。
- **Phase 2D.5** 源码提交 `4fb76dbba388e9fb09e2dd8b8b8ed1d253d92105` 已完成 ARM Release 构建和主机侧测试，但截至归档时 **尚未烧录、尚未实车验证**。
- 上位机 XUC V2 改动已在 `kukuaxing/TJ-advanced` 的 `feat/xuc-board-communication-20260901` 分支，归档提交为 `ed41007501b34ab2e13a125d7d2820a180829a33`。
- 格式塔仿真最终候选为 **v4q**：正反转合计 10/10 个有效回合，300 发、228 次伤害命中，汇总命中率 76.0%。

## 目录

- `firmware/stm32f405-xuc-v2/`：下位机完整源码快照、Git bundle、已烧录 D4 与待验证 D5 构建产物。
- `upper-computer/TJ-advanced-xuc-v2/`：从确切 GitHub 提交取回的 15 个改动文件与提交引用。
- `simulation/gestalt-drive-aim-v4q/`：最终报告、逐轮结果、配置、源码及 10 个 v4q 正反转有效日志。
- `SHA256SUMS.txt`：本仓库除该文件和 `.git` 外所有归档文件的 SHA-256。

## 安全边界

固件与上位机仍处于联调阶段。实车继续测试时应清空弹丸、禁用摩擦轮和拨弹机构、确保云台转动范围无人，并保持 `xuc_allow_shoot: false`。任何“已构建”都不等同于“已烧录”或“已实车通过”。

## 恢复

固件源码可直接使用 `source/`，也可以从 `history/RM_Infantry-Robot-xuc-v2.bundle` 恢复原始 Git 历史。上位机完整工程继续以私有仓库 `kukuaxing/TJ-advanced` 为准，本仓库的 `changed-files/` 是归档提交相对于基线的文件快照。
