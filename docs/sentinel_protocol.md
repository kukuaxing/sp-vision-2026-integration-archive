# Sentinel 固定帧通信

适配依据：[Hohokamk/Sentinel_robot b6f4697756eb8b7c61a6868e450f8d0cda2752ae](https://github.com/Hohokamk/Sentinel_robot/tree/b6f4697756eb8b7c61a6868e450f8d0cda2752ae)，核对 `STM32F405/xuc.h`、`xuc.cpp`、`RC.cpp`、`imu.cpp`、`usart.cpp` 与 `taskslist.cpp`。不自动兼容更旧的32字节版本：固定帧没有版本号，两端必须使用同一布局。

正式配置：`configs/standard.yaml`，`board_protocol: SENTINEL`、`xuc_enable: true`、`xuc_serial_baud: 921600`。通过 `io::XucSender` 的 Sentinel 分支收发；原SP14/20与RMV实现仍供对应旧固件使用。

## 物理连接及模式

UART4，921600，8N1，无流控。`usart.cpp` 配置 **PA0 TX / PA1 RX / AF8**；电控文档仍写PC10/PC11，具体连接以源码与板上引出标注核对。用户已确认连接PA0/PA1对应UART4接口。本文不据此宣称线路或串口已实测通过。

本次源码 `RC_STATE(rc.s[0],rc.s[1])`：双上=AUTO，左上右中=STOP，双中=RESET，双下=ROTATION。双上不存在左右开关映射歧义；其余左右标注沿用电控命名。上行只在AUTO发送，约10Hz。协议没有遥控档位/模式/控制许可字段；视觉端仅从新鲜上行推断发送时处于AUTO，最终接管仍由下位机判定。

`RC::OnPC` 每5ms执行：`mark_pitch += 0.5*pitch_diff`，大Y的6020位置目标累加 `0.5*yaw_diff*8192/(2π)`，死区0.3°；同一误差在新帧到来前会重复使用。这里的误差量不能填成绝对目标角。下位机约250ms无新控制帧后停止视觉摩擦轮/开火/导航。

## 上行：64字节

| 偏移 | 字段 | 类型及单位 |
|---:|---|---|
| 0 | header=0x5A | uint8 |
| 1 | detect_color bit0、reset_tracker bit1、reserved bit2–7 | 0红/1蓝；当前reserved=15 |
| 2 | rsv0 | 2字节 |
| 4 / 8 / 12 | p_roll / p_pitch / p_yaw | float32，度 |
| 16 / 20 / 24 | p_gx / p_gy / p_gz | float32，度/秒 |
| 28 | p_time | uint32，IMU自身采样时间，毫秒 |
| 32 / 33 | p_id / p_valid | uint8；云台身份1，有效0/1 |
| 34 | rsv1 | 2字节 |
| 36 / 40 / 44 | c_roll / c_pitch / c_yaw | float32，度 |
| 48 | c_time | uint32，底盘IMU自身采样时间，毫秒 |
| 52 / 53 | c_id / c_valid | uint8；底盘身份2，有效0/1 |
| 54 | rsv2 | 2字节 |
| 56 / 58 | rx_total / rx_count | uint16，总接收/校验通过计数；会回绕 |
| 60 | rsv3 | 2字节 |
| 62 | CRC16 | uint16，小端，覆盖0–61 |

两块IMU的毫秒时间戳不等于Linux时钟，也没有证据表明两块IMU时钟同步。接收函数保留原始时间、身份及有效标志，不用主机接收时间冒充采样时间。底盘角速度、关节编码器/电机速度、弹速、轮速、发射许可、拨弹状态、出膛事件均未包含在此协议里；`rx_count`也不代表开火接受或弹丸数量。

`io/sentinel/protocol.hpp` 用显式偏移和小端读写，不依赖PC结构体或编译器位域布局。流式解析支持半帧、粘包、噪声及CRC错误后的重新找帧；保留缓冲有界。有效IMU块含NaN/Inf会拒绝；无效IMU块的原始垃圾值不用于控制，但不妨碍确认串口链路。

`sentinel_feedback()`提供双IMU原始数据；`imu_angles_rad()`和`gimbal_gyro_rad_s()`提供转换后的姿态/云台角速度。控制需要新鲜链路、`p_id=1`、`p_valid=1`及持续推进的`p_time`；链路或采样时间超过250ms即失效。底盘有效性独立保留，不伪造底盘速度。

源码将HI91官方roll/pitch以及gx/gy的偏移互换。默认 `sentinel_swap_roll_pitch: true` 按官方字段恢复逻辑轴，原始值始终保留；它不能代替实际安装旋转矩阵。`xuc_imu_*_sign/offset`用于逻辑角映射，方向及既有手眼标定与当前IMU的对应关系仍需单轴只读检查。原版`imu.cpp`的 `rxData[3] << 8 + rxData[2]` 等CRC长度/校验组装还有优先级问题；若`p_valid`持续0，应由电控修复源数据，不放宽视觉门禁。

颜色默认跟随 `detect_color`；重置位的上升沿重置跟踪。桌面显式选择红/蓝会在运行缓存中关闭电控颜色覆盖，不修改正式配置；无人值守启动沿用正式配置的电控颜色策略。

## 下行：40字节

| 偏移 | 字段 | 类型及单位 / 视觉填法 |
|---:|---|---|
| 0 | header=0xA5 | uint8 |
| 1 / 5 | pitch / yaw | float32，度；当前逻辑姿态 |
| 9 / 13 | yaw_diff / pitch_diff | float32，度；目标减当前，yaw取最短转角 |
| 17 | distance | float32，米；有效视觉测距或0 |
| 21 | fireadvice bit0 | uint8；0/1开火电平 |
| 22 | 保留 | 3字节，0 |
| 25 | v_y | float32，0；当前不发送速度前馈 |
| 29 | 保留 | 1字节，0 |
| 30 / 34 | speed_x / speed_y | float32，米/秒；当前视觉无导航命令，均0 |
| 38 | CRC16 | uint16，小端，覆盖0–37 |

先通过IMU映射的逆变换将算法目标角还原到反馈的同一坐标，再计算误差；内部算法仍用弧度。旧SP的 `xuc_yaw_sign` / `xuc_pitch_sign` 不用于此分支，不混用两套转换。所有保留字段清零，40字节一次写入，最多 `sentinel_tx_hz=100`，低于固件队列/5ms解码约200Hz上限；没有导航意图时不发送运动速度。

当前保持 `xuc_allow_control: false`、`xuc_allow_shoot: false`、`auto_fire: false`，只接收。接通串口不会发送零帧或心跳：**AUTO中任何合法控制帧即使fire=0，也会启动摩擦轮**。以后开启控制时，无目标/无效姿态发送一次零误差、不开火的撤销帧，随后停止发送，等固件超时；正常退出亦尽力撤销。其停止摩擦轮不能由视觉单独即时确认。

开启发射后，固定靶稳定资格按电控要求输出电平；固件在上升沿尝试单发，持续300ms后进入约125ms调度的连发。旋转事件仍按预测窗口产生资格。该协议无单发ID/ACK/轮速反馈，不能沿用RMV的有界20发及掉速延迟测量工具，也不能把持续fire=1当作一次可靠单发。旧拨弹反馈复用、ACK门禁、延迟自适应和针对旧内环的外层PI在Sentinel分支停用；既有弹道/时延参数仍只是初值，未重新标定。

## 构建与只读核对

在小电脑恢复连接并完成部署后，由用户执行：

```bash
cd /home/rcsdart/rm27
bash scripts/build.sh standard xuc_board_probe
source scripts/runtime_env.sh
./build/standard configs/standard.yaml --sentinel-check
# 将遥控器切双上后，只读接收；此工具发送0字节。
./build/xuc_board_probe /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 921600 8 --sentinel
```

`--sentinel-check`只查配置，不打开设备；探针分别打印link、pose、原始双IMU字段和收包计数。link=1说明64字节CRC有效上行；pose=1还需有效IMU及推进的时间戳。检查完使用新版STOP档或双中RESET。旧 `configs/calibration.yaml` 的RMV标定流程保留，不能直接向此固件使用。

离线验证：ARM GCC直接提取原始TxPacket做64字节/字段偏移断言；独立Python字节向量交叉核对CRC与字段；Linux私有PTY运行真实XucSender，覆盖只读零输出、拆包、错误帧、角度/角速度转换、±180°回绕、无效位/错身份/时间戳冻结、撤销及恢复。虚拟测试不启动物理机器人；实际在车固件、线路与姿态安装未被这些测试认证。
