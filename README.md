# RoboMaster 视觉与控制工程

本仓库保存当前维护的一份源码。当前小电脑工程目录：`/home/rcsdart/rm27`。
本分支只跟踪源码、配置、构建脚本和文档；模型、SDK库、固件镜像等只在本机保留。
只维护一份工程，源码变更通过 Git 提交记录，不建立带后缀的工程副本或归档分支。

2026-10-31 正式版已完成“导航→任务→返程”实车联调，本轮全程88.92秒。现用视觉策略为 `teammate_prediction`，顺时针预测延时195ms、最短发令间隔225ms、旋转水平门限1.0°、俯仰稳定20ms；导航轮速上限5500rpm，速度上限1.8m/s，并包含加减速平滑和终点控制修正。实车固件待机锁定修正以可重建补丁保存。完整参数、实测边界与后续拓展见[正式版基线](docs/formal_release.md)和[当前调试状态](docs/status.md)。

用户加装临时单发限位后报告一轮30发全中；该轮水平门限为0.4°，最新速度优先档不能沿用该命中结论。成功写出发令、模型预计命中和真实出膛/命中分别记录，离线仿真结果不等同实车命中率。调参及验证边界见[发令门限与提频](docs/fire_gate_and_warmup.md)和[机械延迟调查](docs/shot_latency_investigation.md)。

当前运行配置开启视觉控制、摩擦轮与自动开火。双上=AUTO许可，检测敌方装甲后申请预转、连续丢失5秒撤销摩擦轮保持，左上右中=STOP；STOP持续上传心跳，遥控失联撤权。电机目标转速及实际启动行为以车上固件为准。上下位机接口见[通信说明](docs/sentinel_protocol.md)。

现场实际电控由队友在独立的 `Sentinel_robot-main(1)/Sentinel_robot-main` 工程维护；仓库内 `firmware/sentry` 保留协议适配和回归参考，不代表其PID、电机ID或诊断扩展与当前车上镜像完全一致，见[固件范围](firmware/README.md)。

## 从这里开始

- [当前状态与未完成事项](docs/status.md)
- [队友视觉方案移植及接口边界](docs/teammate_vision_port.md)
- [固定旋转靶仿真与部署结论](docs/simulation_deployment.md)
- [实车门限、首发等待与提频](docs/fire_gate_and_warmup.md)
- [构建、目录和日常维护](docs/development.md)
- [外部依赖与资源校验](docs/dependencies.md)
- [配置与标定状态](docs/configuration.md)
- [不接实车的离线联调与测试](docs/offline_development.md)
- [统一记录与真实检测回放](docs/replay.md)
- [估计器基准与规划时间修复](docs/offline_benchmark.md)
- [回到实车后的接续验收](docs/field_acceptance.md)
- [当前 Sentinel V2 通信](docs/sentinel_protocol.md)
- [既有 RMV 标定与调试协议](docs/rmv_protocol.md)
- [固件选择](firmware/README.md)
- [格式塔仿真](simulation/README.md)
- [后续修改规则](AGENTS.md)

## 目录

| 目录 | 用途 |
|---|---|
| `src/` | 程序入口，正式程序为 `standard` |
| `tasks/auto_aim/` | 识别、目标跟踪、预测及调度 |
| `io/` | 相机、串口及设备接口 |
| `tools/` | 公共算法、一键启动与停止脚本 |
| `calibration/` | 标定与验证工具源码 |
| `configs/standard.yaml` | 唯一正式运行配置 |
| `configs/calibration.yaml` | 标定工具配置，不能当作正式运行配置 |
| `tests/` | 离线及接口测试源码 |
| `assets/`、`librm/` | 本机模型位置（不入库）与源码依赖子模块 |
| `firmware/` | 步兵和哨兵的既有电控源码及交付记录，实际车上版本以当前状态说明为准 |
| `simulation/` | 格式塔仿真成果 |
| `navigation/` | 开机启动、定位就绪、导航与任务编排源码；地图及运行数据另行保留 |
| `docs/` | 当前使用与维护文档 |

`build/`、`logs/`、`data/` 为本机生成内容，不提交 Git。
桌面“启动自瞄”“停止自瞄”仍为现场入口，红蓝选择继续持久化，运行日志保留最近20份。

## 版本来源

视觉源自原小电脑现场工程，整理前 `standard` 与9月12日归档的 SHA-256 一致：
`d97ba9af270b1598c0b543bcc882db2a1cfd4f3cf3f234ed2f25c7c95cfb01c6`。
最初整理未调整算法或现场参数；后续已修复预测时间、规划窗口和弹道数值边界，并加入实车标定与 RMV 调试改动。当前安装外参、弹速估计、经验俯仰修正和临时时延初值见正式配置；验证范围与尚未完成的事项见当前状态说明。
已删除的历史快照和旧标签不恢复，后续进度通过正常 Git 提交记录。
