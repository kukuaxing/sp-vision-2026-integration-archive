# 2026-09-06 最终保留包（更新至 2026-09-10）

这是本轮调车后在 C 盘保留的三项最终成果。历史构建目录、工具链、日志和临时测试文件不属于本包。

## 1. 下位机固件

- 可烧录固件：`firmware/artifacts/stm32f405_xuc_v2_rpm6500.bin`
- 独立烧录回读：`firmware/artifacts/readback_42820.bin`
- 两个文件均为 42,820 字节，SHA-256 完全相同，说明烧录内容与最终构建一致。
- 完整源码：`firmware/source/`
- 最终摩擦轮目标转速：6500。

## 2. 视觉自瞄

- 可执行文件：`vision/binary/standard`
- 源码包：`vision/source-bundle.tar.gz`（截至 2026-09-09 的关键源码，不含二进制）
- 关键源码快照：`vision/source-snapshot/`（含完整 `tasks/auto_aim/`、主程序和 XUC 通信源码）
- 最终现场配置：`vision/config/standard_hikrobot_phase2d5_rpm6500_gyro_oneface_live_trim141_delay250_UNVALIDATED.yaml`
- 一键启停文件：`vision/launcher/`
- 2026-09-09 全部已取回日志：`vision/logs/autoaim-20260909-all.tar.gz`
- 三份关键对比日志：`vision/logs/2026-09-09-key/`
- 日志分析脚本：`vision/analysis/`

最终现场配置保留的关键值：允许下位机控制与发射；yaw 弹着修正 `-0.49`；pitch 滤波 `120 ms`；pitch 最大速率 `25 deg/s`；pitch 误差增益 `0.80`；弹速 `19.36 m/s`；pitch trim `-0.92`。

配置文件名中的 `UNVALIDATED` 是调车期间沿用的历史命名。当前事件开火延迟为 154 ms，发射允许保持开启；最新合并物理命中门尚待下一轮实弹验证。

`vision/binary/standard` 仍是 2026-09-06 历史二进制，不能代表 2026-09-09 的算法迁移。最新源码曾在小电脑编译通过，但归档时未连接小电脑，最终二进制本体待补采。详见 `AUTOAIM_MIGRATION_FIELD_REPORT_20260910.md`。

## 3. 格式塔仿真

- 最终程序：`simulation/src/gestalt_drive_aim_v4q_centered_handoff_20260902.cpp`
- 最终配置：`simulation/configs/tongji_outpost_converged_oneface16_driveaim_rel30_pitch0_30.yaml`
- 汇总、报告和最终十次运行日志均保存在 `simulation/`。

## 完整性

关键文件的 SHA-256 见 `SHA256SUMS.txt`。旧归档曾经提交到本 Git 仓库的内容仍可从 Git 历史或远端仓库恢复；清理掉的 Codex 临时工作目录不在最终保留范围内。

## 4. 当前系统与后续路线

- 完整交接记录：[`HANDOFF_20260907.md`](HANDOFF_20260907.md)
- 当前系统、已知问题、开源方案比较和分阶段技术路线：[`AUTOAIM_STATUS_AND_ROADMAP_20260908.md`](AUTOAIM_STATUS_AND_ROADMAP_20260908.md)
- 算法迁移、三轮实弹日志结论与最新版本边界：[`AUTOAIM_MIGRATION_FIELD_REPORT_20260910.md`](AUTOAIM_MIGRATION_FIELD_REPORT_20260910.md)

截至 2026-09-10，`vision/source-snapshot/`、`vision/source-bundle.tar.gz`、最终 YAML 和日志已包含本机取回的 2026-09-09 最新增量。仅 `vision/binary/standard` 仍为 2026-09-06 历史版本，后续回到实验室后补齐最新构建产物。
