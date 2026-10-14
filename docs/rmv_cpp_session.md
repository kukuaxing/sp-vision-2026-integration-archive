# C++ RMV 主动会话

`io/rmv/session.hpp` 提供仅依赖 C++17 标准库的事件驱动 `io::rmv::Session`。它与串口、线程、相机 SDK 和正式 SP 主循环分离。20261004 交付固件实现了 RMV 原始 IMU/关节/参数接口；标定读取小 Y 原始姿态，正式 SP 主循环仍需另行适配。

## 接口和接入顺序

调用方在同一个线程中调用所有方法，或用同一个锁保护完整会话。时间均为同一 PC 单调时钟的微秒，禁止混用墙钟、MCU 时钟和相机设备时钟。

1. 调用 `begin(random_nonce, now_us)`，默认要求基础能力 `0x1f`。每次新连接生成不同的随机非零 nonce。HELLO 必须带 `ACK_REQUEST`（flags=1）；成功仍由专用 HELLO_REPLY 完成。当前标定适配只要求小 Y 原始 IMU 能力 `1<<10`。
2. 每轮把收到的字节交给 `ingest(bytes, now_us)`，调用 `tick(now_us)`，再取 `take_outgoing(now_us)` 返回的报文并立刻按顺序写出。
3. 默认自动维护心跳和同步：心跳间隔 100 ms；初始至少 8 个有效同步样本，之后每 250 ms 更新一个，保持整个 8 样本窗口在 3 s 内。调用周期建议不超过 5 ms。自动维护不会请求控制模式、发送目标或发射。
4. 等同步完成且收到有效的支架姿态后，显式调用 `request_mode(3, 3, now_us)` 请求小 Y/P 标定，或 `request_mode(2, 3, now_us)` 请求世界系视觉控制。
5. 请求 ACK 成功且 STATUS 确认所有权和许可后，`control_ready(now_us)` 才返回 true。调用 `aim(yaw, pitch, now_us, reference)`；reference=0 为世界系，2 为关节系且仅允许标定。默认目标有效期 80 ms。
6. 结束调用 `stop(now_us)`，取出并发送 STOP，读取 ACK。端口错误、无法继续发送或退出进程时调用 `disconnect()`；它只清空本地会话，不代表已经向电控发出 STOP。链路失效后的保持由电控执行。

最小循环结构（串口实现由调用者提供）：

```cpp
io::rmv::Session session;
session.begin(nonce, monotonic_us());
while (running) {
  session.ingest(read_available_bytes(), monotonic_us());
  session.tick(monotonic_us());
  for (const auto& bytes : session.take_outgoing(monotonic_us()))
    write_immediately_in_order(bytes);
  // 读取 last_result()，由业务显式请求模式和最新目标。
}
```

`take_outgoing()` 转移发送责任。传输层不能在后台积压目标；当前 `SerialTransport` 遇到短写会关闭端口并抛出错误，调用方随即让会话失效，不续写或重放无法确认的半帧。不能把上一次连接尚未发送的数据带到新端口。已经返回的报文无法再被 Session 撤销。

为使确定性测试逐步控制同步，可传 `SessionOptions{false}` 关闭自动同步，再逐次调用 `request_time_sync(now)`。自动心跳仍由 `tick()` 维护。通常应保留自动同步。

## 失败行为

- HELLO 检查专用响应标志、准确长度、nonce、非零 session、所需能力、保留能力位、负载上限和轴数量。保存最近 64 个 nonce 和 boot/session 对以发现近期复用；固件仍必须保证同 boot 内 session 永不复用。boot 变化清空全部旧状态。
- ACK 必须同时匹配请求序号、消息类型、当前会话及响应标志。`BAD_SESSION` 仅允许当前 session 或 session=0 的匹配 ACK，并立即清空会话。专用响应请求不能用成功 ACK 冒充完成。
- 同时最多一个 HELLO、TIME 或模式/STOP 配置请求。`pending()` 表示结果仍未确定。100 ms 无对应响应标记超时；超时不会伪造成功，也不自动重新申请控制权。
- 显式 `retry(now)` 只重发已超时的 HELLO、MODE、STOP，最多两次，保留完全相同的序号、时间戳和负载。TIME 用新采样重做；AIM 永不重发。`last_result()` 保存最近结果，区分 accepted、rejected、timeout、disconnected 和 protocol_error；新请求不把旧结果改成成功。
- STOP 优先撤销尚未发出的目标和未完成的模式请求。STOP 后本地控制许可保持关闭，必须重新请求模式；旧 MODE ACK 不能恢复它。坐标 epoch 或安装结构版本变化同样要求重新请求模式。
- 500 ms 没有可接受的回复/遥测，或 500 ms 没有送出心跳，清空会话。先检查失效再接收迟到包，迟到状态不能复活旧会话。PC 时间倒退同样失效。
- 尚未移交给传输层的报文超过 5 ms 未发送，全部丢弃并断开会话；连续产生的未发送 AIM 只保留最新一条。发送队列最多 8 帧。
- 产生目标同时要求新鲜 STATUS、控制所有权、物理许可、无故障、同步有效、支架姿态有效且不超过 100 ms。不使用缺失/无效姿态外推，不以模式 ACK 替代实际许可。

本实现暂未提供 C++ 参数枚举/事务、发射、底盘、前馈或大 Y 协调控制；参数工具使用 Python RMV 客户端。它也不负责安装外参是否正确、图像时间的真实含义、MCU 断线后的实际保持策略。上述项仍需要正式固件和现场验收。

## 已有离线验证

运行 `python3 scripts/check_rmv_firmware.py --firmware-zip <交付压缩包路径>`，直接用 SHA256 校验后的交付固件 `rmv_core.h` 做内存对接。测试正确 HELLO、持续同步及静止小 Y 姿态，并重现 flags=0 的 BAD_VALUE 拒绝和缺失能力的 UNSUPPORTED 拒绝；标定错误报告保留主控错误码。只有两个头文件临时提取到系统临时目录，测试结束删除，不打开物理设备、不修改固件。

`tests/rmv/session_test.cpp` 使用独立构造的报文检查：握手拒绝/复用、不同 boot、错误 ACK 关联、逐字节分包与 CRC 恢复、同步回复匹配和高延迟拒绝、MODE 的 ACK 与 STATUS 区别、目标合并与有限期限、STOP 抢占与重试、超时原字节重发、旧会话/过期遥测/时钟倒退、无心跳失效，以及超过 8 s 的自动心跳与时钟维护。测试始终没有设备 I/O；模拟通过不能说明实车控制性能。

`tests/rmv/session_peer.cpp` 仅在显式传入 `--stdio-offline` 时启动，通过标准输入/输出与 Python 模拟器互通。首行是 `1000000 none -`；后续各行格式为 `host_now_us action rx_hex`，没有输入字节时用 `-`。action 为 none、mode、aim、stop、disconnect、connect、retry 或 sync。mode 固定请求双轴标定，aim 固定请求关节目标 `(0.1, 0.05)` rad；这些数值仅用于模拟。它返回单行 JSON：active、session、ready、mode（无状态时 -1）、result（无结果时 -1）、completion、action_accepted、pending、wires。wires 是应交给离线 Board 的十六进制报文数组。进程内没有串口打开路径、真实时钟等待或电机接口。

## Linux 私有 PTY 验收

在 Linux 工程根目录运行 `nice -n 19 python3 scripts/check_rmv_pty.py`。脚本只以 `g++ -O0` 顺序编译会话测试和仓库已有 serial 后端，产物在 `build/offline-linux/`；整个脚本及其子进程以 nice 19、ionice idle 运行。它不运行 CMake，不启动或修改导航、ROS、服务，不安装软件。

脚本创建自己的伪终端，把 C++ `Session + SerialTransport` 与 Python `Board` 接起来。`tests/rmv/pty_peer.cpp` 只接受显式 `--pty-offline /dev/pts/<数字>`，并在打开任何端口前拒绝 `/dev/ttyUSB0` 等实串口路径。验收覆盖真实内核字节流中的分包、首条回复 CRC 损坏、插入噪声、握手、自动时钟同步、标定模式、连续目标、STOP 回执和 SAFE 状态。进程 5 s 内不能完成就报错退出。

2026-09-26 在本机 WSL Ubuntu-22.04 实际通过：15 条模拟目标全部收到，STOP 和 SAFE 确认；30 次主机报文读取、155 次碎片写入，噪声及 CRC 错误后恢复。每次运行的计数可能因系统调度不同而变化，报告在 `logs/rmv/pty_acceptance.json`。这证明 Linux 串口后端与协议会话在 PTY 上互通，不代表 USB 电气、真实曝光时间精度、IMU 安装关系或云台响应已经验收。
