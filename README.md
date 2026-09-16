# RoboMaster 视觉与控制工程

本仓库保存当前维护的一份源码。小电脑正式目录：`/home/rm/sp_vision`。
本分支只跟踪源码、配置、构建脚本和文档；模型、SDK库、固件镜像等只在本机保留。
仓库已清除旧提交与旧标签，只保留当前最终版，不保留历史备份或带后缀的副本。

## 从这里开始

- [当前状态与未完成事项](docs/status.md)
- [构建、目录和日常维护](docs/development.md)
- [外部依赖与资源校验](docs/dependencies.md)
- [配置与标定状态](docs/configuration.md)
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
| `firmware/` | 步兵和哨兵两个硬件目标的当前固件 |
| `simulation/` | 格式塔仿真成果 |
| `docs/` | 当前使用与维护文档 |

`build/`、`logs/`、`data/` 为本机生成内容，不提交 Git。
桌面“启动自瞄”“停止自瞄”仍为现场入口，红蓝选择继续持久化，运行日志保留最近20份。

## 版本来源

视觉源自原小电脑现场工程，整理前 `standard` 与9月12日归档的 SHA-256 一致：
`d97ba9af270b1598c0b543bcc882db2a1cfd4f3cf3f234ed2f25c7c95cfb01c6`。
这次整理不调整算法、弹道、外参或发射参数。新安装外参仍待标定。
旧快照、旧标签和本地历史备份已删除，不能再从本仓库恢复旧版本。
