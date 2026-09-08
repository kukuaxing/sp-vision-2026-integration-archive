# 2026-09-06 最终保留包（更新至 2026-09-08）

这是本轮调车后在 C 盘保留的三项最终成果。历史构建目录、工具链、日志和临时测试文件不属于本包。

## 1. 下位机固件

- 可烧录固件：`firmware/artifacts/stm32f405_xuc_v2_rpm6500.bin`
- 独立烧录回读：`firmware/artifacts/readback_42820.bin`
- 两个文件均为 42,820 字节，SHA-256 完全相同，说明烧录内容与最终构建一致。
- 完整源码：`firmware/source/`
- 最终摩擦轮目标转速：6500。

## 2. 视觉自瞄

- 可执行文件：`vision/binary/standard`
- 源码包：`vision/source-bundle.tar.gz`
- 关键源码快照：`vision/source-snapshot/`
- 最终现场配置：`vision/config/standard_hikrobot_phase2d5_rpm6500_gyro_oneface_live_trim141_delay250_UNVALIDATED.yaml`
- 一键启停文件：`vision/launcher/`

最终现场配置保留的关键值：允许下位机控制与发射；yaw 弹着修正 `-0.49`；pitch 滤波 `120 ms`；pitch 最大速率 `25 deg/s`；pitch 误差增益 `0.80`；弹速 `19.36 m/s`；pitch trim `-0.92`。

配置文件名中的 `UNVALIDATED` 是调车期间沿用的历史命名；本包按 2026-09-06 最后一次固定靶实测后的参数保存。

## 3. 格式塔仿真

- 最终程序：`simulation/src/gestalt_drive_aim_v4q_centered_handoff_20260902.cpp`
- 最终配置：`simulation/configs/tongji_outpost_converged_oneface16_driveaim_rel30_pitch0_30.yaml`
- 汇总、报告和最终十次运行日志均保存在 `simulation/`。

## 完整性

关键文件的 SHA-256 见 `SHA256SUMS.txt`。旧归档曾经提交到本 Git 仓库的内容仍可从 Git 历史或远端仓库恢复；清理掉的 Codex 临时工作目录不在最终保留范围内。

## 4. 当前系统与后续路线

- 完整交接记录：[`HANDOFF_20260907.md`](HANDOFF_20260907.md)
- 当前系统、已知问题、开源方案比较和分阶段技术路线：[`AUTOAIM_STATUS_AND_ROADMAP_20260908.md`](AUTOAIM_STATUS_AND_ROADMAP_20260908.md)

截至 2026-09-08，`vision/source-snapshot/`、最终 YAML 和启动器已包含 2026-09-07 的最新视觉增量。由于整理时不在实验室、无法连接小电脑，`vision/binary/standard` 与 `vision/source-bundle.tar.gz` 仍为 2026-09-06 版本；后续回到实验室后再补齐最新构建产物。
