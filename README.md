# RoboMaster 视觉与控制工程

本仓库保存当前维护的一份源码。当前小电脑工程目录：`/home/rcsdart/rm27`。
本分支只跟踪源码、配置、构建脚本和文档；模型、SDK库、固件镜像等只在本机保留。
只维护一份工程，源码变更通过 Git 提交记录，不建立带后缀的工程副本或归档分支。

2026-10-14 更新：已保存标定采集、相机颜色修正、同步姿态与关节反馈、旋转靶预测、MPC 轨迹控制和有界单发/轮速掉速诊断的代码与回归测试。10 月 7 日的 RMV 实车记录属于当时的调试固件；用户随后提供的新电控仓库为 [Hohokamk/Sentinel_robot](https://github.com/Hohokamk/Sentinel_robot)。当前正式配置仍选择 RMV，尚未适配新电控的固定帧协议；旧档位、单发事件和延迟测量结果不能直接当作新固件的已验证能力。接续状态见 [当前状态](docs/status.md)。

## 从这里开始

- [当前状态与未完成事项](docs/status.md)
- [构建、目录和日常维护](docs/development.md)
- [外部依赖与资源校验](docs/dependencies.md)
- [配置与标定状态](docs/configuration.md)
- [不接实车的离线联调与测试](docs/offline_development.md)
- [统一记录与真实检测回放](docs/replay.md)
- [估计器基准与规划时间修复](docs/offline_benchmark.md)
- [回到实车后的接续验收](docs/field_acceptance.md)
- [视觉—电控RMV串口协议](docs/rmv_protocol.md)
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
| `docs/` | 当前使用与维护文档 |

`build/`、`logs/`、`data/` 为本机生成内容，不提交 Git。
桌面“启动自瞄”“停止自瞄”仍为现场入口，红蓝选择继续持久化，运行日志保留最近20份。

## 版本来源

视觉源自原小电脑现场工程，整理前 `standard` 与9月12日归档的 SHA-256 一致：
`d97ba9af270b1598c0b543bcc882db2a1cfd4f3cf3f234ed2f25c7c95cfb01c6`。
最初整理未调整算法或现场参数；后续已修复预测时间、规划窗口和弹道数值边界，并加入实车标定与 RMV 调试改动。当前安装外参、弹速估计、经验俯仰修正和临时时延初值见正式配置；验证范围与尚未完成的事项见当前状态说明。
已删除的历史快照和旧标签不恢复，后续进度通过正常 Git 提交记录。
