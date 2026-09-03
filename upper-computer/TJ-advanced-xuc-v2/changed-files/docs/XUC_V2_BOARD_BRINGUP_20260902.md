# XUC V2 下位机接板与自瞄通信检查

更新时间：2026-09-02

## 1. 已确认的系统分工

学长已确认新版链路采用以下分工：

```text
IMU -> 下位机 -> XUC上行 -> 小电脑
小电脑：视觉检测、PnP、坐标融合、Tracker、预测、目标角计算
小电脑 -> XUC下行目标yaw/pitch -> 下位机 -> 云台
```

小电脑不再直接读取原IMU串口/CAN；旧CBoard可通过`cboard_enable: false`停用。

## 2. 新版二进制协议

所有结构体使用1字节对齐，当前代码通过`static_assert`检查包长和CRC偏移。

### 下位机发送、小电脑接收：20字节

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | head | ASCII `SP` |
| 2 | 1 | mode | 0空闲、1自瞄、2小符、3大符 |
| 3 | 1 | robot_id | 3红方、103蓝方 |
| 4 | 4 | bullet_speed | `float`，弹速 |
| 8 | 2 | bullet_count | `uint16_t`，累计弹数 |
| 10 | 4 | imu_pitch | `float`，rad |
| 14 | 4 | imu_yaw | `float`，rad |
| 18 | 2 | crc16 | 低字节在前 |

### 小电脑发送、下位机接收：14字节

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | head | ASCII `SP` |
| 2 | 1 | control | 是否允许云台控制 |
| 3 | 1 | shoot | 是否开火 |
| 4 | 4 | yaw | `float`，目标yaw，rad |
| 8 | 4 | pitch | `float`，目标pitch，rad |
| 12 | 2 | crc16 | 低字节在前 |

## 3. 接板前已完成的验证

- 14/20字节协议编译期检查通过。
- CRC错误拒绝、噪声重同步、半包拼接通过。
- mode、robot ID、弹速、弹数和IMU字段解析通过。
- 500 ms反馈失联后状态失效，恢复后重新有效。
- 合法反馈前`control=0`，失联后撤控，恢复后重新允许。
- 安全配置全程`shoot=0`。
- 独立只接收探测工具通过PTY测试。
- `standard`完整构建通过。

虚拟探测的闭合统计：

```text
RX_BYTES=2023
HEADER_CANDIDATES=101
CRC_ERRORS=1
FIELD_ERRORS=0
VALID_PACKETS=100
XUC_VIRTUAL_PROBE_RESULT=PASS
```

其中2023字节由100个合法包、1个损坏包和3个噪声字节组成。

## 4. 尚需实物确认的接口事实

当前暂按旧链路约定使用：

- 460800 baud、8N1、无流控；
- CRC初值`0xFFFF`、反射多项式`0x8408`；
- CRC覆盖`crc16`字段前的全部字节；
- CRC低字节先发送；
- 主机与MCU均采用小端IEEE-754 float。

若探测工具持续出现CRC错误，应优先向学长确认这些事实，不能通过修改坐标算法解决。

新版包没有roll、IMU采样时间戳或相机触发计数。当前上位机使用收到的最新yaw/pitch构造ZY姿态，能够用于初次跟随验收，但不等价于旧链路的完整四元数时间插值。

## 5. 安全默认值

`configs/standard_hikrobot.yaml`必须首先保持：

```yaml
cboard_enable: false
xuc_require_feedback_for_control: true
xuc_allow_control: false
xuc_allow_shoot: false
```

`xuc_allow_control`只有在接收、零心跳、方向和急停检查完成后才可临时设为`true`。

在完成实弹安全验收之前，`xuc_allow_shoot`始终保持`false`。

## 6. 接板操作顺序

### 6.1 机械与电气安全

1. 机器人架空，确保云台不会撞击人员、桌面或线缆。
2. 取出弹丸，断开摩擦轮和拨弹机构动力。
3. 准备物理急停或随时断开云台电源。
4. 不启动完整自瞄。

### 6.2 构建工具

```bash
cd ~/sp_vision_25-xuc-board-20260901
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target xuc_board_probe xuc_link_test standard -j2
```

### 6.3 枚举真实串口

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/ 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

优先使用稳定的`/dev/serial/by-id/...`路径，不要把`ttyUSB0/1`顺序写死。

设置本次终端变量：

```bash
XUC_PORT=/dev/serial/by-id/实际设备名
fuser -v "$XUC_PORT" 2>/dev/null || true
```

若有其他进程占用串口，先正常停止对应进程，不要同时打开同一设备。

### 6.4 第一步：只接收

```bash
mkdir -p logs/xuc-bringup/2026-09-02
./build/xuc_board_probe "$XUC_PORT" 460800 10 \
  | tee logs/xuc-bringup/2026-09-02/receive-only.log
```

通过条件：

- `RX_BYTES`持续增加；
- `VALID_PACKETS`持续增加；
- `FIELD_ERRORS=0`；
- `CRC_ERRORS`不持续增加；
- IMU角度有限且手动小幅转动时连续变化；
- `XUC_PROBE_RESULT=PASS`。

### 6.5 第二步：全零安全心跳

只接收通过后执行：

```bash
./build/xuc_board_probe "$XUC_PORT" 460800 10 --safe-heartbeat \
  | tee logs/xuc-bringup/2026-09-02/safe-heartbeat.log
```

此模式固定发送`control=0`、`shoot=0`、`yaw=0`、`pitch=0`。云台和发射机构不得动作。

### 6.6 第三步：完整程序禁控运行

把`xuc_serial_port`改为已确认的`/dev/serial/by-id/...`，并再次确认：

```yaml
xuc_allow_control: false
xuc_allow_shoot: false
```

然后短时运行：

```bash
timeout --signal=INT --kill-after=3s 15s \
  stdbuf -oL -eL \
  ./build/standard configs/standard_hikrobot.yaml \
  | tee logs/xuc-bringup/2026-09-02/standard-control-locked.log
```

检查`[SYNC][XUC]`姿态、识别和Tracker日志。此时传输层仍强制`control=0`、`shoot=0`。

### 6.7 第四步：方向与跟随

1. 使用只接收探测工具，分别手动小幅增加云台yaw和pitch，记录IMU读数方向。
2. 必要时调整`xuc_imu_yaw_sign`和`xuc_imu_pitch_sign`。
3. 确认目标命令方向；必要时调整`xuc_yaw_sign`和`xuc_pitch_sign`。
4. 机器人继续架空且无弹，将`xuc_allow_control`临时改为`true`。
5. 从目标接近画面中心、小角度开始运行，随时准备急停。
6. 验证跟随、目标丢失、进程退出、串口拔出和反馈中断时云台退出控制。
7. 测试结束后把`xuc_allow_control`恢复为`false`。

## 7. 诊断计数解释

| 现象 | 优先检查 |
|---|---|
| `RX_BYTES=0` | 设备名、供电、接线、串口占用、波特率 |
| 有字节但`HEADER_CANDIDATES=0` | 波特率、包头、线路噪声 |
| `CRC_ERRORS`持续增加 | CRC算法、包长、字节序、结构体版本 |
| `FIELD_ERRORS`增加 | 字段布局、浮点格式、非法mode或NaN |
| `VALID_PACKETS`增加但很快失效 | 下位机发送周期、串口丢包、500 ms超时 |
| 姿态连续但方向相反 | IMU sign配置和安装方向 |
| 姿态正确但云台反向 | 目标yaw/pitch sign配置 |

## 8. 暂停条件

出现以下任一情况立即保持`xuc_allow_control=false`并停止动作测试：

- CRC错误持续增加；
- IMU出现NaN、突跳或单位明显不符；
- 云台方向错误、持续旋转或越过机械安全范围；
- 拔串口或停止程序后云台仍保持自瞄控制；
- `shoot`或发射机构出现任何非预期动作。
