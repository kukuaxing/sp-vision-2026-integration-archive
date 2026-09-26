# 不接实车的离线开发与联调

2026-09-26：已补齐通信会话/传输、统一记录回放、真实检测器文件推理、实际几何/估计/规划回归和合成对照。本轮没有连接实车或小电脑，没有更改正式配置、现场固件和启动入口。`standard` 仍使用 SP；RMV 尚未作为现场链路启用。

## 本机验证结果

| 检查 | 实际结果与边界 |
|---|---|
| 原生 CMake/CTest | Windows/MSVC、Release，23 个验收入口全部通过；断言保持启用 |
| Python | RMV 45 项、记录回放 42 项、指标 5 项；共 92 项，91 通过、1 项实际符号链接测试因 Windows 创建权限跳过 |
| RMV 黄金报文 | 28 种、1616 个分包位置；Python 与 C++ 编解码字节一致 |
| C++ ↔ Python | 3 组跨进程交互：持续同步、丢 ACK 原请求重试、重启、断流、旧目标到期、STOP |
| Linux 私有 PTY | 实际内核串口字节流收到 15 条模拟目标；分包、噪声、坏 CRC 恢复、STOP/SAFE 通过；不代表 USB 或电机性能 |
| Solver → Target → Aimer | 真实源码、9 个合成视角；最大位置往返误差约 1.42e-5 m；不验证安装外参真实性 |
| Target → Planner → TinyMPC | 真实源码；修复后窗口中心误差约 -3.20e-10 rad，非零执行延迟只补一次 |
| 合成估计器基准 | 16 场景、37 组对照、88,800 行；固定种子重复运行 CSV 字节一致 |
| 实际 YOLO26 | 本机已有模型与历史视频，8 帧、9 个检测；两个独立模型实例结果完全一致；空图/黑图检查通过。单线程耗时中位约 28.8 ms，仅属于本机 |

完整日志在 `logs/offline_acceptance.log`、`logs/rmv/pty_acceptance.json`、`logs/benchmark/`、`logs/replay/`。模型、视频、图像和生成结果不提交 Git。GitHub Actions 工作流已配置；本轮没有推送或执行远端 CI。

## 使用入口

Python 3.10+。基础验收不需要 OpenCV、相机 SDK、ROS、模型或网络：

```text
python scripts/check_offline.py
python -m tools.rmv demo
python -m tools.rmv inspect logs/rmv/offline_demo.jsonl
python -m tools.replay inspect data/replay/demo/events.jsonl
python -m tools.replay replay data/replay/demo/events.jsonl
python scripts/benchmark_offline.py
```

基础脚本在 Windows 自动发现 MSVC，在 Linux 使用 g++/clang++。`--python-only` 明确跳过 C++，不能称为完整验收。所有编译串行执行，产物仅在 `build/`。

真实视觉数学模块使用独立的 CMake 离线构建入口，跳过相机 SDK、设备驱动、CUDA 和 ROS 发现。它编译工程中的 Solver、Target、Aimer、Planner 和 TinyMPC，并非另写一套算法。

```text
python scripts/check_offline.py --with-vision
```

Windows 本机已有 vcpkg/OpenVINO，完整构建命令为：

```text
python scripts/check_offline.py --with-inference --vcpkg-root D:/Dev/vcpkg --openvino-dir C:/Users/32751/AppData/Local/Programs/Python/Python313/Lib/site-packages/openvino/cmake
```

本机 Python 完整路径为 `C:/Users/32751/AppData/Local/Programs/Python/Python313/python.exe`。其他电脑需使用自己的依赖路径；脚本不自动安装或改写全局环境。

`--with-inference` 只构建文件推理工具。用户显式指定文件后才能运行实际检测回放：

```text
python scripts/replay_detector.py --input assets/demo/test.mp4 --output data/replay/新的采集目录 --max-frames 8 --stride 10 --repeat-check
```

输出目录必须尚不存在，防止覆盖采集数据。Windows 子进程低于普通优先级；推理和解码各限制为单线程。没有同步姿态的视频只生成图像、检测与耗时记录，不生成目标、控制或命中数据。详见[记录与回放](replay.md)。

Linux 可选运行 `python3 scripts/check_rmv_pty.py`。脚本低优先级单进程编译并创建自己的 `/dev/pts/`，拒绝真实串口路径；它不操作导航、ROS 或服务。导航仿真仍运行时，不应启动本工程的全量 Linux 编译或不受限推理争用资源。

## 本轮修复的实际问题

- YawESKF 把 0.40 s 预测截断到 0.10 s。现在完整积分预测时长，保留工作量上限和数值失效处理。
- Planner 负时间回退只影响主 EKF，未同步回退航向子滤波器。现在从同一状态按相对时间只读采样，中央时刻严格一致。
- Planner optional 入口重复补执行延迟；现在统一补一次，并支持显式时钟用于确定性回放。
- 弹道无解时数值未初始化、非法速度/距离可能产生 NaN；现在有确定的无解结果、稳定低弧根和往返检验，Planner 在无解时提前返回。
- 角度归一化对无限值或极大输入可能长循环；现在常数时间处理并保持原有 (-π,π] 约定。
- Python 丢 MODE ACK 后仍可能允许目标，未决参数提交被其他请求覆盖；现在锁定控制与待重试请求，迟到 ACK/旧响应均匹配上下文。
- C++ 旧模式 ACK 跨 epoch/安装版本解锁；现在要求原坐标上下文仍有效。
- Solver 外参向量过短可能越界；现在先检查长度。另增明确的 G→W 直接姿态入口，避免 RMV 四元数重复应用旧安装变换。
- 修正串口库 Windows 头文件顺序/端口前缀、fmt 枚举兼容及 YOLO 基类析构，算法输入不再强制依赖 SocketCAN。

修复前后的基准与取舍见[离线基准](offline_benchmark.md)。没有依据合成排名改变正式滤波、弹道或发射参数。MPC 仍不是正式 standard 当前启用的控制链路。

## 通信接入边界

协议、Python 虚拟主控和客户端、C++ 会话及真实串口适配分别位于：

- `docs/rmv_protocol.md`
- `tools/rmv/protocol.py`、`simulator.py`、`client.py`
- `io/rmv/core.hpp`、`session.hpp`、`transport.hpp`、`serial.hpp`

[C++ 接入说明](rmv_cpp_session.md)定义单线程 pump、自动同步/心跳、5 ms 待发队列期限、消息关联和部分写入失败策略。Python 参数事务目前仍使用离线内存传输；不能拿演示命令直接插 USB 操作车辆。

虚拟主控仅模拟固定底座上的小 Y/P 和协议语义。机械范围为夹具值，运动模型没有惯量/摩擦/真实 PID；不模拟发射、底盘、大小 Y 协同或真实 Flash 掉电。回零明确返回不支持。未测的控制抖动等诊断为占位值。

## 已到达的现场边界

继续决定实际参数或放行控制，需要以下输入，不能从模拟器推断：

1. 电控实现并确认 RMV、实车参数表、能力位及故障/保持语义；USB 硬件时序、拔插与重启验收。
2. 确认 UART3 小 Y IMU 是否同时随 P 运动，以及相机支架、枪口、关节的完整安装关系；完成新外参。
3. 在小电脑原有相机 SDK/OpenVINO 环境全量构建、核对曝光时间/缓存/端到端延迟。
4. 获取本车同步的图像、检测、姿态、目标、指令和已采用反馈，验证真实噪声、丢帧、选面和跟踪。
5. 实测轴响应、弹速、出膛延迟和命中偏差，再选择预测模型、滤波强度、前馈与发射策略。

具体顺序和交付材料见[现场接续验收](field_acceptance.md)。这些项通过前，不能把当前“离线通过”写成“新车已调好”。
