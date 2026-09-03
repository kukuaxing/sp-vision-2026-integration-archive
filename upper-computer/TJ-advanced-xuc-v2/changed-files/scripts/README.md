# sp_vision_25 自瞄 配套脚本

本文件夹汇总了 `sp_vision_25` 自瞄程序的**配套脚本与工具**，并给出从**标定**到**运行**的完整操作流程。

> 这些文件是从原位置**复制**过来的（`~/build.sh`、`~/imu_parser/`、`~/serial_send/` 等），原文件保留不动。
> 部分脚本内含**绝对路径 / 硬编码**，真机使用前需按实际环境修改，见文末「路径修正清单」。

## 一、目录结构与各部分作用

| 文件 | 作用 |
|---|---|
| `build.sh` | **一键重新构建**整个项目：删 `build/` → `cmake -B build` → `make -C build -j$(nproc)`。改代码后必跑（在项目根目录执行 `bash scripts/build.sh`）。 |
| `start_imu_bridge.sh` | **IMU→CAN 转发桥**一键启动。把接在上位机 USB 串口（默认 `/dev/gimbal` @115200）的维特智能 IMU 数据，转发到虚拟 CAN `can0` / ID `0x150`，供标定工具 `capture` 和运行时 `CBoard` 读四元数。含防重复启动检测 + 自动建 vcan 接口。 |
| `imu_parser.cpp` / `imu_to_can.cpp` | 桥的源码。`imu_parser` 解析维特智能 `5A A5` IMU 协议（82 字节帧，四元数/欧拉角/加速度）；`imu_to_can` 把欧拉角打包成 CAN `0x150` 帧（yaw/pitch/roll 半精度大端 + data[6] IMU 计数）。需重编译时：`g++ -O2 -o imu_to_can imu_to_can.cpp`。 |
| `serial_send.sh` / `serial_send.cpp` | **通用串口调试工具**：打开指定串口（硬编码 `/dev/ttyUSB2` @115200）每 2s 发一条 10 字节命令、共 5 次。用于验证串口链路 / 下位机是否在听。⚠️ 是通用串口发包样例，**不是 XUC 协议**。 |
| `start_spvision.sh` | **启动自瞄** `standard` 程序（后台运行，日志写 `~/spvision.log`）。⚠️ 目前指向旧路径 `/home/rm/test/sp_vision_25-main` 和 `configs/sentry.yaml`，需改成当前项目路径和实际配置。 |
| `autostart.sh` | **开机自启**入口（`screen` 起 `./watchdog.sh`，日志到 `logs/`）。⚠️ 指向 `~/Desktop/sp_vision_25`，需改。 |
| `imu_parser_README.md` | imu_parser 的原始说明（IMU 协议细节）。 |
| `README.md` | 本文档。 |

**不在本文件夹、但流程要用的构建产物**（在 `~/sp_vision_25/build/`）：`capture`（标定采图）、`calibrate_camera`（内参标定）、`calibrate_handeye`（手眼标定）、`standard`（自瞄主程序）。

## 二、操作流程（从标定开始）

### 0. 前置准备（一次性）
- **硬件接线**
  - IMU（维特智能）→ 上位机 USB 串口，识别为 `/dev/gimbal`（→ `ttyUSB1`），115200，100Hz
  - 海康相机 → USB3，`lsusb` 见 `2bdf:0001`
  - 下位机（TWLInfantry STM32）UART4 → USB 转 TTL → 上位机 `/dev/ttyUSB0`，460800
- **串口权限**（当前用户加入 `dialout` 组，重新登录生效）
  ```bash
  sudo usermod -aG dialout $USER
  ```
- **确认硬件在**
  ```bash
  ls /dev/ttyUSB* /dev/ttyACM*
  lsusb | grep 2bdf
  ```

### 1. 构建项目
```bash
cd ~/sp_vision_25
bash scripts/build.sh        # 等价于 cmake -B build && make -C build -j$(nproc)
```

### 2. 启动 IMU 桥（标定 `capture` 和运行时都要用 IMU）
```bash
cd ~/imu_parser
./start_imu_bridge.sh                 # 前台运行，Ctrl+C 停止
# 可自定义：./start_imu_bridge.sh /dev/ttyUSB1 115200 can0 0x150
```
- **校验数据干净**（静止云台，相邻帧值一致、间隔约 10ms）：
  ```bash
  timeout 3 candump -td can0,150:1FFF
  ```
- ⚠️ 坑：CAN ID 必须写 `0x150`（写 `150` 会被桥当成十进制 → `0x96`，capture 收不到）；同一串口只允许一个桥（防重复检测已内置）。

### 3. 相机内参标定
棋盘格**固定**（靠墙/三角架，不要手持移动），云台转到多角度：
```bash
cd ~/sp_vision_25
./build/capture configs/calibration_hikrobot.yaml -o assets/img_with_q   # 显示 Chessboard OK 后按 s 保存，q 退出
./build/calibrate_camera -c configs/calibration_hikrobot.yaml assets/img_with_q
```
回填输出的 `camera_matrix` / `distort_coeffs` 到 `configs/calibration_hikrobot.yaml`（内参要先于手眼标定）。

### 4. 手眼标定（相机 ↔ 云台外参）
棋盘固定、只转云台（yaw 左中右、pitch 上中下铺开，采 10~15 组）：
```bash
./build/calibrate_handeye -c configs/calibration_hikrobot.yaml assets/img_with_q
```
回填 `R_gimbal2imubody` / `R_camera2gimbal` / `t_camera2gimbal` 到 config。
- **合格线**：旋转残差 mean < 2°，平移 < 20 mm。
- ⚠️ 若 `R_camera2gimbal` 偏角 roll ≈ ±180° 而实机 raw 图像**正立**：是绕光轴 180° 伪影（多因标定运动以 yaw 为主导致 roll 欠约束），需右乘 `Rz(180°)` 修正，不要手工改 Euler 角。根治：重采标定时让云台同时动 pitch+roll。

### 5. 构建正式 runtime 配置
`configs/calibration_hikrobot.yaml` 是标定/调试专用，缺 `enemy_color/yolo*/tracker*/aimer*/shooter*` 等运行时键，`standard` 直接跑会崩。
**以 `configs/standard3.yaml` 为骨架**建一份本车的完整 config，替换：
- `camera_matrix` / `distort_coeffs`（本相机内参）
- `R_gimbal2imubody` / `R_camera2gimbal` / `t_camera2gimbal`（本车外参）
- `camera_name: "hikrobot"` + 曝光/增益
- `xuc_*` 键（若用 TWLInfantry 下位机，见 §6）

### 6. 运行自瞄
```bash
./build/standard <完整配置路径>
```

### 7. XUC V2链路（TWLInfantry 下位机，UART4 @460800）
小电脑与下位机使用新版双向协议，所有角度固定为rad：
- **下行14B `RxPacket_TJ`**：`'SP' + control + shoot + 目标yaw/pitch + crc16`。
- **上行20B `TxPacket_TJ`**：`'SP' + mode + robot_id + bullet_speed + bullet_count + IMU pitch/yaw + crc16`。
- 串口配置：`xuc_enable` / `xuc_serial_port` / `xuc_serial_baud`。
- 目标角方向：`xuc_yaw_sign` / `xuc_pitch_sign`；IMU方向：`xuc_imu_yaw_sign` / `xuc_imu_pitch_sign`。
- 首次接板保持`xuc_require_feedback_for_control: true`、`xuc_allow_control: false`、`xuc_allow_shoot: false`。
- 优先使用稳定的`/dev/serial/by-id/...`设备路径，不依赖`ttyUSB`编号。
- 接板时先运行只接收探测：
  ```bash
  ./build/xuc_board_probe /dev/serial/by-id/<实际设备> 460800 10
  ```
- 完整接板顺序、诊断计数和解锁条件见`docs/XUC_V2_BOARD_BRINGUP_20260902.md`。

## 三、已知问题 / 待办
1. **XUC V2尚未真机验收**：虚拟协议、接收诊断和安全状态机已通过；仍需用新版主控确认串口设备、CRC、角度方向、反馈超时及云台跟随。
2. **完整 runtime config 未建**（见 §5）。
3. `start_spvision.sh` / `autostart.sh` 路径陈旧（指向 `/home/rm/test/sp_vision_25-main` / `~/Desktop`）。

## 四、路径修正清单（复制过来后要改）
| 文件 | 需要改 |
|---|---|
| `start_spvision.sh` | `cd /home/rm/test/sp_vision_25-main` → 实际项目路径；`configs/sentry.yaml` → 实际 config |
| `autostart.sh` | `cd ~/Desktop/sp_vision_25` → 实际项目路径 |
| `serial_send.cpp` | 串口设备 `/dev/ttyUSB2`、波特率、命令字节按需改，重编译：`g++ -o serial_send serial_send.cpp` |
