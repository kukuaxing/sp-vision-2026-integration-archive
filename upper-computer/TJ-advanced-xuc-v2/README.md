# TJ-advanced 上位机 XUC V2 快照

完整工程的权威来源是私有仓库 `kukuaxing/TJ-advanced`。

- 基线分支：`test/xuc-virtual-link`
- 基线提交：`b8ef6d536ef0903a3b602d619bcc71db968b74fe`
- 功能分支：`feat/xuc-board-communication-20260901`
- 归档提交：`ed41007501b34ab2e13a125d7d2820a180829a33`
- 相对基线提交数：8
- 相对基线改动文件数：15

`changed-files/` 的原始核心文件通过 GitHub Contents API 从上述精确提交取回，后续又加入 2026-09-04 小电脑现场快照。它们覆盖 14 字节下行/20 字节上行协议、下位机 IMU 接入、旧 CBoard 链路停用、安全状态机、相机标定与颜色修正、像素 Pitch 闭环和双轴跟随。

这份目录用于离线审阅和交接；实际开发、合并与构建仍应在 TJ-advanced 完整仓库对应分支进行。

## 2026-09-04 实车现场快照

`changed-files/` 已补充小电脑现场工作树中与相机标定、颜色修正、XUC 通信、像素 Pitch 闭环和双轴跟随相关的文件。这些现场文件尚未合并回完整 TJ-advanced 仓库，因此本归档提交是当前权威快照。

- 最终双轴配置：`changed-files/configs/standard_hikrobot_phase2d5_joint_acceptance.yaml`
- Pitch 单轴回退配置：`changed-files/configs/standard_hikrobot_phase2d5_pitch_acceptance.yaml`
- 双轴实车日志和结论：`field-results/2026-09-04/`
- 开火仍禁用：`auto_fire: false`、`xuc_allow_shoot: false`

## 提交序列

1. `f41cfce` 适配新版 14/20 字节串口协议
2. `7e9b2fc` 接入下位机 IMU 并停用旧 CBoard 链路
3. `9a5a197` 验证接板安全状态机
4. `c942d2f` 新增真实下位机接板探测工具
5. `a1f27da` 增加串口接收诊断计数
6. `d448d49` 增加云台控制显式解锁
7. `50ef2b7` 添加新版下位机接板操作手册
8. `ed41007` 统一 V2 接板说明和安全默认值
