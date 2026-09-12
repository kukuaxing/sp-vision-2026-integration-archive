# 手眼标定操作简报

- 日期：2026-08-03
- 适用范围：`~/sp_vision_25`（视觉项目）+ `~/imu_parser`（IMU 转发，独立程序）

---

## 1. 结论速览

- **崩溃问题已修复**：之前 `./build/calibrate_handeye -c configs/calibration_hikrobot.yaml hae`
  报 `std::__ios_failure: Is a directory`，根因是 OpenCV `CommandLineParser` 不认 `-c value`
  这种空格写法，已加参数规整层修复，现在**空格 / 等号写法都能用**。
- **数据问题未解决**：手眼标定要求**棋盘格全程固定、只转云台**。手持移动棋盘会让残差升高
  （当前 28 组数据旋转残差 mean 9°，不合格），标定结果**不可回填**。
- **画面乱跳 / 显示 non 已定位根因**：同时启动了**两个 `imu_to_can` 桥进程**抢同一个串口
  `/dev/ttyUSB1`，IMU 帧被瓜分打碎 → capture 左上角 yaw/pitch/roll 乱跳、出现 non。
  已给 `start_imu_bridge.sh` 加**重复进程检测**（重复启动会被拒绝）；CAN ID 必须写 `0x150`，
  写 `150` 会被解析成 `0x96`（capture 收不到帧）。操作见 §3①②，坑见 §5-8~10。
- 一套合格的手眼标定，按本文 §3 操作，再按 §4 校验即可。

---

## 2. 本次排查要点

### 2.1 崩溃根因（`std::__ios_failure`）

1. 本项目 OpenCV 4.5.4 的 `cv::CommandLineParser` **只认 `-x=value` / `--xxx=value`**，
   `-x value` 会被当成"布尔开关"：选项值被塞成字符串 `"true"`，而紧跟的路径变成位置参数。
2. 于是 `YAML::LoadFile(config_path)` 实际读的是 `YAML::LoadFile("true")`。
3. 工作目录下恰好有个叫 `true` 的文件夹 → `ifstream` 打开目录、`read()` 返回 EISDIR → 抛
   `std::__ios_failure`，未捕获 → `terminate` + 核心转储。

> `./true` 目录就是之前用 `-o assets/img_with_q`（空格写法）跑 capture 时被解析成输出到
> `"true"` 而误建的。确认无用可删。

### 2.2 代码修复（本次改动）

新增 `tools/cmdline.hpp` 的 `tools::normalize_argv()`，在构造 `CommandLineParser` 前把
`-x value` 规整成 `-x=value`。5 个标定工具已接入：

- `calibration/calibrate_handeye.cpp`
- `calibration/calibrate_camera.cpp`
- `calibration/calibrate_robotworld_handeye.cpp`
- `calibration/capture.cpp`（`-o` 同理）
- `calibration/split_video.cpp`

### 2.3 数据层面的坑（手眼标定成败的关键）

| 数据特征 | 后果 |
|---|---|
| 四元数全是 `1 0 0 0`（`require_cboard=false` 或 IMU 未接） | 所有云台姿态相同 → 退化解 `R=I, t=0`，偏角 -90/90/0，**无意义** |
| 棋盘格在每张图之间移动/换俯仰 | 手眼方程 `Aᵢ·X·Bᵢ=常数` 不成立 → 残差高，`R_camera2gimbal` 不可信 |
| 棋盘固定 + 云台多角度 | 正确做法，残差应 < 2° |

---

## 3. 操作流程（从启动 IMU 开始）

### 前置（一次性）

- IMU 通过 USB 串口（CH340）连上位机，识别为 `/dev/gimbal`（→ `ttyUSB1`），
  波特率 115200，帧率 100Hz，维特智能 `5A A5` 协议，数据含四元数 w,x,y,z。
- IMU 转发程序在 `/home/rm/imu_parser`（独立于视觉项目，`imu_parser.cpp` / `imu_to_can.cpp`）。

### 步骤

**① 启动 IMU → CAN 转发**

```bash
cd /home/rm/imu_parser
./start_imu_bridge.sh
```

- 默认参数：`/dev/gimbal 115200 can0 0x150`
  （可自定义：`./start_imu_bridge.sh /dev/ttyUSB1 115200 can0 0x150`）
- **CAN ID 必须写 `0x150`**，不要写 `150`（后者会被桥解析成 `0x96`，capture 收不到）；
- 首次运行会 `sudo` 创建虚拟 CAN 接口 `can0`（需输密码）；
- **同一串口只允许一个桥**：若脚本提示"已有 imu_to_can 进程在运行，拒绝再启动一个！"，
  先 `kill <PID>` 结束旧桥再重跑（两个桥抢串口会打碎 IMU 帧，画面角度乱跳 + non）；
- **前台运行，Ctrl+C 停止**；采集期间保持运行。

**② 确认转发正常且数据干净（10 秒，强烈建议）**

```bash
ip link show can0                     # 确认 can0 存在且 UP
timeout 3 candump -td can0,150:1FFF   # 静止云台抓几帧，3 秒自动停
```

静止云台时正常帧如下——**相邻帧值一致、间隔约 10ms**：

```
 (000.000000)  can0  150   [8]  41 E3 34 6D C2 15 07 00
 (000.009901)  can0  150   [8]  41 E3 34 6D C2 15 08 00
 (000.010197)  can0  150   [8]  41 E3 34 6D C2 15 09 00
```

出现以下任一条 = 桥在发垃圾，**先修再采集**：

- 帧间隔变成 20~70 微秒的"突发"（多进程抢串口 / 帧同步错位）；
- 相邻帧 yaw/pitch/roll 字节（`data[0..5]`）乱跳，或出现 `7E 00` / `7F 00`
  （半精度 ±Inf / NaN）；
- capture 画面左上角角度乱跳、出现 `non`。

> 注意：`data[6]` 计数器正常递增**不代表数据正确**——那是桥自己的发送计数，不是 IMU 姿态。

**③ 固定标定板（铁律）**

- 棋盘**位置 + 朝向全程纹丝不动**：靠墙 / 垫桌沿 / 三脚架，不要手举着换来换去；
- 之后只转云台，绝对不碰棋盘。

**④ 采集标定数据**

```bash
cd ~/sp_vision_25
./build/capture configs/calibration_hikrobot.yaml -o=assets/img_with_q
```

- 画面显示"Chessboard OK"且云台角正常后按 `s` 保存，`q` 退出；
- **采 10~15 组**即可，云台 yaw 左中右、pitch 上中下都铺开；
- 逐张核对画面左上角 yaw/pitch/roll 是否与实际云台角一致（不一致说明安装矩阵/IMU 有问题）。

**⑤ 相机内参标定（仅首次或换镜头时）**

```bash
./build/calibrate_camera -c=configs/calibration_hikrobot.yaml assets/img_with_q
```

- 逐张按空格；
- 把输出的 `camera_matrix` / `distort_coeffs` 回填到
  `configs/calibration_hikrobot.yaml`（内参要先于手眼标定，手眼会用到）。

**⑥ 手眼标定**

```bash
./build/calibrate_handeye -c=configs/calibration_hikrobot.yaml assets/img_with_q
```

- 逐张按空格核对云台角；
- 输出 `R_gimbal2imubody` / `R_camera2gimbal` / `t_camera2gimbal`。

**⑦ 校验（按 §4），合格后回填配置。**

---

## 4. 结果校验标准

| 指标 | 合格线 |
|---|---|
| 一致性残差（旋转） | mean **< 2°**（当前数据 9°，不合格） |
| 一致性残差（平移） | < 20 mm（当前 99 mm，不合格） |
| 偏角 yaw / pitch / roll | 通常几度以内；roll 若 ≈ ±180° 需单独核对（见 §5） |
| 逐图云台角 | 与实机云台角度一致 |

> 残差即"棋盘固定时每张图反推的 `R_target2world` 的分散度"。
> 需要复算时把数据目录给我即可。

---

## 5. 注意事项 / 坑清单

1. **命令行参数**：现在 `-c value`、`-c=value`、`--config-path=value` 都能用
   （已修复）；修复前只有 `=` 形式可靠。
2. **棋盘必须固定**：手拿且每次移动，残差会到 5~10°+，标定不可信。
3. **内参先于手眼**：`camera_matrix`/`distort_coeffs` 必须先回填，手眼依赖内参做 solvePnP。
4. **单位四元数数据**（`require_cboard=false` 或 IMU 未启动时采的）**只能做内参**，不能做手眼。
5. **t_camera2gimbal 意义有限**：工具把 `t_gimbal2world` 写死为 0，主要看旋转 `R_camera2gimbal`。
6. **`./true` 目录**是历史 bug 误建的，可删。
7. **git 显示全仓库大量文件已修改**是行尾（换行符）问题，与本次改动无关，勿误提交。
8. **同一串口只能有一个 `imu_to_can`**：两个桥进程抢读会把 IMU 帧打碎，画面角度乱跳 + non。
   已给 `start_imu_bridge.sh` 加重复检测，重复启动会被拒绝。
9. **CAN ID 必须 `0x150`**：桥里 `strtoul(..., 0)` 会把 `150` 当成十进制 = `0x96`，
   capture 听 `0x150` 会收不到任何帧。启动转发时参数要写 `0x150`。
10. **计数器正常 ≠ 数据正常**：`data[6]` 是桥自己的发送计数，帧同步坏了它也照样递增。

---

## 6. 遗留事项

- [ ] 按 §3 重采（棋盘固定），重跑手眼，残差达标后回填 `R_camera2gimbal`。
- [ ] 当前新数据偏角 roll ≈ 173°，待残差合格后核对：
  可能相机真实安装偏转，或 `R_gimbal2imubody`（现 `[-1,0,0,0,-1,0,0,0,1]`）/
  `R_gimbal2ideal` 约定与实机不符，需现场比对逐图云台角确认。
