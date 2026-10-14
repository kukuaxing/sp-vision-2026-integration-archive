# 视觉—电控 USB 串口通信约定

2026-10-07当前固件2026100708已按用户持续授权烧录，完整读回一致、0x08020000起参数Flash保留并重启。镜像102448字节，SHA25648a64c63011eabae54e918a714c8e4964c114268b6f721e7ff943d5b685f085a，记录RCS_code_rmv/data/flash/20261007_202018/result.json。用户确认拨弹轮七槽，保持42130counts单格；0707日志18193489765的末段过冲后，实际速度近0，目标已要求归位，旧积分仍令电流远离目标2–3秒。0708仅在低速、120counts范围外、限斜率参考指向目标而净电流背离目标时清除阻碍归位的积分；正常制动和到位范围内负载补偿保留。速度400rpm、电流3000、积分限幅1500、到位120counts/35rpm/30ms、制动40ms及6秒超时保持。新双向回归旧版失败、修正版通过；原8组固件检查/互通、ARM、真实host→Device及9组PTY通过，小电脑兼容部署完成。首次8秒只读检查记录1次格式错误（CRC/超时0），保留原记录；随后独立8秒复查三轴在线、故障/CRC/格式/超时0。合并诊断799帧，默认恢复ACK成功，摩擦轮/拨弹停止、拨弹IDLE/命令电流0。未启动发射。19.8m/s、-1.91°、85ms及6500rpm保持；改善末段归位及短时间双弹的实际效果尚待用户5次复测，不宣称出膛延迟已标定。证据logs/deployment/feeder-0708-update.json。以下0707及更早为历史记录。

调试固件2026100704新增、2026100705/2026100706/2026100707沿用私有 **0x0109 SHOOT_TRACE**，默认不发送，通过SUBSCRIBE启用，最高200Hz；原0x0104的52字节布局保持。这是临时台架诊断扩展，不是下面正式协议提案的交付声明，也不复用预留能力位。0704诊断配置为STATUS20Hz、POSE25Hz、JOINT25Hz、SHOOT_STATE0Hz、SHOOT_TRACE200Hz，加诊断/服务各1Hz共28,564B/s；结束恢复20/100/100/20/0Hz。字段均小端，负编码器计数和电流必须有符号解析，电流为原生单位而非安培。

| 字节偏移 | 类型 | 含义 |
|---|---|---|
| 0 | uint64 | MCU快照时间，微秒 |
| 8、12 | uint32各1 | 上次请求ID、实测转速有效位（bit0/1双轮、bit2拨弹） |
| 16、20、24 | float各1 | 左轮、右轮、拨弹转速，rpm |
| 28、32、36 | int32各1 | 拨弹编码器位置、当前绝对目标、剩余误差，编码器计数；0707起误差始终为目标减位置，旧版中断IDLE时可能残留上轮误差 |
| 40 | float | 拨弹速度命令，rpm |
| 44、48 | int32各1 | 拨弹原生电流命令、反馈电流 |
| 52、56、60 | uint32各1 | 左轮、右轮、拨弹反馈年龄，微秒 |
| 64、65 | uint8各1 | 拨弹机构状态、电机控制模式 |
| 66 | uint16 | 拨弹故障 |
| 68 | uint32 | bit0单发已武装、bit1连续已武装、bit2轮速就绪、bit3堵转、bit4反馈故障、bit6测试；高16位堵转重试次数 |
| 72 | uint32 | 编码器单发周期完成计数 |

机构状态0..9分别为STOPPED、WAIT_READY、READY、SINGLE_FEED、CONTINUOUS_FEED、JAM_CONFIRM、JAM_REVERSE、RECOVER_WAIT、TEST_FEED、FAULT；电机模式0..6分别为SPD、POS、ACE、wheel_left、wheel_right、BRAKE、IDLE。负右轮转速正常。总载荷76字节，完整帧106字节。此帧未提供实测弹速或弹丸出膛事件；SHOT_EVENT=2仍只代表编码器拨弹完成。

版本：RMV 1.0 提案，2026-09-26。状态：待双方确认和实现，**不是当前固件已经支持的协议**。

本文件是项目内维护版本。离线客户端、模拟器和验证范围见[离线开发说明](offline_development.md)，报文生成使用`python -m tools.rmv vectors logs/rmv/serial_test_vectors.json`。模拟器只实现文档注明的子集，不代表全部交车条目已完成。

适用：小电脑通过 USB 转串口连接主控；小 Y、P、大 Y 云台。已知安装接线：小 Y IMU 接 UART3，大 Y IMU 接 UART5。接在小 Y 上，不等于一定随 P 轴一起运动；交车时必须确认安装关系。

## 1. 双方要先达成的约定

**视觉给目标，电控负责安全地执行；电控把实际姿态、执行情况和可调参数完整返回。**

希望做到的是：电控交车后，视觉可以独立进行相机标定、姿态对齐、跟踪参数调整、云台响应测试和约定范围内的控制参数调整。要实现这一点，下面的接口必须真正做完并通过验收，不能仅保留空字段。

| 方向 | 交车前必须具备 | 用途 |
|---|---|---|
| 电控 → 视觉 | 相机支架完整四元数、角速度、采样时间、有效性 | 外参、姿态补偿、图像与姿态对齐 |
| 电控 → 视觉 | 小 Y/P 实际角度、速度、采用的目标、在线/回零/故障状态 | 识别跟踪误差、饱和、限速、机械故障 |
| 电控 → 视觉 | 模式、控制权、允许控制的轴、拒绝原因、实际执行序号 | 知道指令有没有执行，以及为什么没执行 |
| 视觉 → 电控 | 有期限的目标角度；明确坐标系和目标有效性 | 正常瞄准和标定运动 |
| 双向 | 握手、能力查询、时钟同步、心跳、订阅频率 | 稳定接入、断线恢复、时序分析 |
| 双向 | 可调参数的枚举、读取、范围查询、成组修改、保存 | 后续通过小电脑调参，避免反复改常量、烧录 |
| 电控 → 视觉 | 固件版本、参数版本、安装结构版本、通信与控制周期统计 | 复现问题，避免版本不一致 |

摩擦轮/拨弹、真实弹速、出膛检测、大 Y 联动和底盘运动属于选配能力；**如果后续计划用到，应在本次交车范围里勾选并实现**。预留消息号不等于已经支持。新硬件驱动、新控制算法和未约定的新功能仍可能需要修改固件。

## 2. 职责边界与交付物

电控负责电机驱动、闭环控制、回零、硬限位、合理的速度/加速度上限、失联处理、遥控器优先级、故障保护和发射最终许可。不得把电流环/PID 实时闭环放到 USB 链路上。

视觉负责识别、跟踪、预测、弹道、相机标定、目标角度、目标有效期、串口客户端、图像与姿态时间对齐，以及调参和日志工具。视觉不直接发电机电流，也不通过参数接口解除硬件安全限制。

交车时电控提供：本协议的支持项清单；固件构建版本；参数导出；传感器/关节安装表；零位与正方向说明；遥控器开关对应模式；USB 串口接线、电平和端口识别方式；回零和故障恢复步骤；实际控制频率及响应测试记录。

视觉提供：本协议与参考编解码器、正常/错误报文样例、上位机连接和调参工具、带图像时间/目标序号的日志格式及联调验收记录。随附 `serial_protocol.py` 仅为线格式参考与离线自测，**不是已经完成的机器人上位机适配程序**。

## 3. 物理链路与带宽

- 初始约定：460800 baud、8 数据位、无校验、1 停止位、无流控，全双工；双方先确认现有转接器和 MCU UART 支持。
- USB 转串口的信号电平、TX/RX/GND 和供电连接由电控确认。不要依据 USB 接口外形推断串口电平。
- 小电脑同一时间仅一个进程打开串口；Linux 使用稳定的设备路径或 udev 别名，不依赖 `/dev/ttyUSB0` 的枚举顺序。
- 正式端口只跑 RMV 报文，不混发裸 HI91、旧 SP、printf 或 VOFA 文本。调试输出用其他端口或结构化诊断报文。
- 460800、8N1 的理论容量为每方向 46080 字节/秒，建议稳态占用低于 70%。推荐相机姿态 100 Hz、关节 100 Hz、状态 20 Hz、发射状态 20 Hz、诊断 1 Hz；目标 100 Hz。原始 IMU 默认不订阅。
- 两轴情况下，上述 MCU → PC 默认消息约 24 kB/s；三轴约 27 kB/s。订阅更多传感器时必须重新算带宽。921600 只能作为双方验证后的配置，不在运行中自行切换。
- MCU 采用非阻塞收发；实时遥测可丢弃旧帧保留最新帧，不得积压数秒；配置回执不能被遥测长期饿死。

## 4. 统一帧格式

所有整数小端；浮点为 IEEE754 float32；无 C 结构体填充，禁止直接发送未经核对的 `sizeof(struct)`。角度 rad、角速度 rad/s、加速度 rad/s²、时间 µs、距离 m。

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | magic | byte[4] | ASCII `RMV1`，十六进制 `52 4D 56 31` |
| 4 | major | u8 | 1 |
| 5 | minor | u8 | 0 |
| 6 | msg_type | u16 | 消息号 |
| 8 | payload_len | u16 | 0～256 |
| 10 | flags | u16 | bit0 请求 ACK，bit1 响应；其他位必须为 0 |
| 12 | seq | u32 | 发送方每发一帧加 1，允许回绕 |
| 16 | session_id | u32 | 握手生成的会话号；握手请求为 0 |
| 20 | sender_time_us | u64 | 发送方本机单调时钟，启动以来的 µs |
| 28 | payload | byte[N] | 下文规定的负载 |
| 28+N | crc16 | u16 | 小端 CRC |

总长度 30+N。CRC 覆盖从 magic 开始的全部头部和负载，**不含 CRC 自身**。

CRC-16/CCITT-FALSE：poly=0x1021，init=0xFFFF，refin=false，refout=false，xorout=0；`123456789` 的结果为 `0x29B1`。这是本提案选定的算法，不得直接沿用旧 SP 的 CRC 实现而不核对参数。

接收端必须支持分包、粘包和噪声；先查 magic 和长度，再校验 CRC。长度越界、版本不支持、标志位错误或 CRC 错误时，丢弃当前候选帧第一个字节，重新查找 magic。未收齐的候选帧最多等待 50 ms，然后重新同步。不要把一次 UART DMA 回调或一次 USB read 当成一帧。

minor 0 仅接受本文规定的准确负载长度；新增字段使用新消息号或协商版本，不能默默改变旧结构体。未知消息若请求 ACK，返回 UNSUPPORTED；其他未知消息丢弃并计数。所有保留字段写 0。有效浮点字段禁止 NaN/Inf。无效遥测浮点填 0，并清除有效位；**0 本身不能代表有效性**。

## 5. 握手、时间与消息目录

下文结构使用 Python `struct` 简写：B=u8、H=u16、I=u32、Q=u64、i=i32、f=float32，全部带 `<` 小端前缀。`3s` 是 3 个显式保留字节。

| 消息 | 方向 | 格式 / 长度 | 功能 |
|---|---|---|---|
| 0x0001 HELLO | PC→MCU | `<QIBBH` / 16 | nonce、required_caps、min_major、max_major、保留 |
| 0x8001 HELLO_REPLY | MCU→PC | `<QQIIIIHBB` / 36 | nonce、boot_id、fw_version、caps、param_rev、geometry_rev、max_payload、imu_count、axis_count |
| 0x0002 TIME_SYNC | PC→MCU | `<Q` / 8 | t1：PC 发送时间 |
| 0x8002 TIME_REPLY | MCU→PC | `<QQQ` / 24 | t1、t2：MCU 收帧时间、t3：MCU 入发送队列时间 |
| 0x0003 HEARTBEAT | PC→MCU | `<B3s` / 4 | host_state：0待机/1运行/2故障、保留 |
| 0x0004 SUBSCRIBE | PC→MCU | 见下文 | 配置遥测频率 |
| 0x00FF ACK | 双向 | `<IHHII` / 16 | ack_seq、ack_msg_type、result、detail、revision |
| 0x0101 STATUS | MCU→PC | `<8BIIIHHII` / 32 | 系统和指令执行状态 |
| 0x0102 POSE | MCU→PC | `<BBHIQQIffffffffff` / 68 | 姿态 |
| 0x0103 JOINT_STATE | MCU→PC | 16+32×N | 关节状态 |
| 0x0104 SHOOT_STATE | MCU→PC | `<QIIffffIffIHH` / 52 | 可选，发射机构状态 |
| 0x0105 SHOT_EVENT | MCU→PC | `<IBBHQI` / 20 | 可选，发射事件 |
| 0x0106 SERVICE_STATE | MCU→PC | `<IBBHQIB3s` / 24 | 回零/故障恢复操作状态 |
| 0x0107 CHASSIS_STATE | MCU→PC | `<QIIffffff` / 40 | 可选，平面底盘里程计 |
| 0x0108 DIAGNOSTICS | MCU→PC | `<10I` / 40 | 通信和控制周期统计 |
| 0x0200 MODE_REQUEST | PC→MCU | `<BBHI` / 8 | mode、axis_mask、保留、frame_epoch |
| 0x0201 AIM_SETPOINT | PC→MCU | `<IQQBBHffffff` / 48 | 目标角度及有效期 |
| 0x0202 STOP_REMOTE | PC→MCU | `<HH` / 4 | reason、保留 |
| 0x0203 SHOOT_REQUEST | PC→MCU | `<IBBHIQ` / 20 | 可选，单次发射/取消 |
| 0x0204 CHASSIS_SETPOINT | PC→MCU | `<QQIBBHfff` / 36 | 可选，底盘速度 |
| 0x0205 SERVICE_REQUEST | PC→MCU | `<IBBHI` / 12 | 清除已消失的锁存故障、回零、取消回零 |
| 0x0300 / 0x8300 | 双向 | 第10节 | 参数列表 |
| 0x0301 / 0x8301 | 双向 | 第10节 | 参数描述和值 |
| 0x0302～0x0304 | PC→MCU | 第10节 | 参数暂存、提交、撤销 |

### 5.1 会话与能力

PC 每次重新连接生成新的随机 nonce。MCU 在 HELLO_REPLY 头部返回非零 session_id，回显 nonce；boot_id 每次 MCU 重启必须变化，session_id 在同一 boot_id 内不得复用。fw_version 为双方维护的 u32 构建编号，交付清单记录对应 Git 提交。

HELLO 请求头使用 session_id=0、flags=ACK_REQUEST（1）。20261004 交付固件对缺失此标志的请求返回 BAD_VALUE（4）；成功使用 HELLO_REPLY 专用响应，不以成功 ACK 完成握手。

会话成功前只接受 HELLO。新会话清除旧远程目标、旧发射请求和暂存参数，不自动进入视觉模式。电机的本地姿态保持由电控负责。已有远程控制尚有效时，新 nonce 返回 BUSY，不抢占控制；同 nonce 握手重发只重发原响应，不创建新会话。

required_caps 超出 MCU 能力时返回 UNSUPPORTED，不建立可控制会话。握手拒绝的 ACK 使用 session_id=0；PC 应以新 nonce 降低需求重新连接或直接报错。旧会话数据一律不执行。

caps 位定义：0相机支架姿态、1关节反馈、2世界系瞄准、3关节系标定、4参数接口、5速度前馈、6加速度前馈、7发射控制与状态、8实测弹速、9出膛传感器、10小Y原始IMU、11大Y原始IMU、12底盘速度与里程计、13大小Y协调、14图像硬件触发时间。其他位保留。

**基础交车要求 bits 0～4 全部为 1。** 握手、同步、心跳、状态、诊断、回执和订阅是所有设备的必备功能，不靠可选位关闭。能力位仅在完整实现、测试通过时置 1。bit14 的触发事件格式本版未定义，必须另行定稿后才能对外声明。

### 5.2 时间同步与新鲜度

PC 收到 TIME_REPLY 时记录 t4。MCU 相对 PC 的时钟偏移估计为 `((t2-t1)+(t3-t4))/2`；往返链路时间估计为 `(t4-t1)-(t3-t2)`。连接后至少取 8 组，优先使用小往返时间样本，之后每秒更新。t2 尽量在完整帧接收完成处采样，t3 在实际入发送队列前采样。

同步算法给出估计值，不意味着 USB 提供硬件级曝光同步。PC 记录同步不确定度；建议不确定度大于 5 ms 或超过 3 s 没有成功同步时停止产生新的运动/发射请求。图像时间应尽量使用曝光时间，视觉保存姿态历史并在图像时间处插值，不能始终拿“最新姿态”代替。

遥测的 sample_time_us 必须是**该数据被采样/接收为新样本时的 MCU 时间**，不能每次转发都刷新旧数据时间。POSE 同时上报源计数和时间质量；IMU 自带时钟须映射到 MCU 时钟，不能直接把两个时钟值相减。

HEARTBEAT 10 Hz；500 ms 不见合法心跳，撤销远程许可、停止新的发射并结束会话。心跳不续期运动目标。目标在自己的 deadline 到期就停止继续跟随；不必等待心跳超时。电控处理 P 轴时应按预先验证的本地保持/减速策略处理，不能简单断电造成下坠。遥控器接管优先于上位机。

### 5.3 回执与错误码

HELLO、TIME、参数列表/描述使用专用响应，flags.bit1=1。其余配置/模式/参数/发射请求必须设置 ACK_REQUEST，MCU 返回 ACK。AIM/CHASSIS 高频成功指令默认不逐帧 ACK，由 STATUS 与关节反馈报告执行；拒绝原因在 STATUS 中更新，若请求 ACK 则另发 ACK。CRC 错误不回 ACK，避免对损坏的 seq 作出回应。

| result | 含义 |
|---:|---|
| 0 | OK，已接受；不等于已经到位或已经出膛 |
| 1 | UNSUPPORTED，功能/消息/字段不支持 |
| 2 | BAD_VERSION |
| 3 | BAD_LENGTH |
| 4 | BAD_VALUE，字段/保留位/浮点不合法 |
| 5 | BAD_SESSION |
| 6 | WRONG_MODE，无当前模式或控制权许可 |
| 7 | NOT_READY，未回零/传感器无效/时钟或机构未就绪 |
| 8 | STALE，指令已过期、重复或顺序落后 |
| 9 | OUT_OF_RANGE，超过允许范围 |
| 10 | BUSY |
| 11 | CONFLICT，参数版本或坐标系 epoch 不符 |
| 12 | HARDWARE_FAULT |
| 13 | RESOURCE_LIMIT，订阅带宽/事务资源不足 |

ACK.detail 默认 0；参数失败填第一个失败 key；订阅失败填消息号；控制失败填 constraint_flags；revision 返回当前参数版本。专用响应的请求不再额外返回成功 ACK。

seq 是两个方向分别递增的报文序号。判断新旧使用模 2³² 半区间规则：`0 < (a-b) mod 2^32 < 2^31`。AIM 与 CHASSIS 分别维护最后接受的命令序号；重复/落后目标不得续期。模式/参数/发射等操作超时重试使用原请求，不生成新的动作 ID；电控保证重复操作不会执行第二次。AIM 只发新目标，不补发过时目标。

配置/模式/STOP/SERVICE/参数/发射等有副作用的操作统一串行处理：每个会话记录最后操作seq、请求内容及响应，相同seq同内容重发返回原响应，同seq不同内容返回CONFLICT，更旧seq返回STALE；首次请求即记录最终接受/拒绝结果，重试不会因模式变化突然执行。PC拿到回执后再发下一项操作；高频AIM和心跳不推进这个操作序号水位。新STOP无需等待旧操作完成，可用更新seq优先处理并撤销尚未完成的远程动作。

## 6. 系统状态与关节反馈

### 6.1 STATUS：32 字节

按顺序：`mode:u8, owner:u8, available_axis_mask:u8, enabled_axis_mask:u8, homed_axis_mask:u8, remote_online:u8, vision_permit:u8, fire_permit:u8, fault_mask:u32, last_received_aim_seq:u32, last_applied_aim_seq:u32, last_aim_result:u16, constraint_flags:u16, param_rev:u32, geometry_rev:u32`。

mode：0 SAFE、1 MANUAL、2 VISION、3 CALIBRATION、4 FAULT。owner：0无、1遥控、2视觉。轴 mask bit0 小Y、bit1 P、bit2 大Y。布尔量仅 0/1。last_received_aim_seq 表示最近完成校验且属于当前会话的 AIM（可能被拒绝）；last_applied_aim_seq 只在控制循环真正采用目标时更新。首条命令之前二者为 0，PC 不发送 seq=0 的控制指令。

fault_mask：bit0 IMU失效、1小Y故障、2P故障、3大Y故障、4遥控失联、5远程失联、6回零失败、7硬限位触发、8控制超时、9发射机构故障、10底盘故障；其余保留。

constraint_flags：bit0角度限位、1速度限幅、2加速度限幅、3目标过期、4遥控接管、5传感器无效、6未回零、7发射被禁止、8时间/epoch不符。目标角度越界默认拒绝整条指令；运行时速度/加速度限幅允许执行，但必须置位并报告采用目标。电控不能悄悄把所有问题都返回 OK。

### 6.2 JOINT_STATE：16+32×N 字节

头部 `<BBHQI`：count、保留、sample_flags、sample_time_us、frame_epoch。N=1～3。sample_flags bit0 时间有效，其余保留。

每轴 `<BBHffffffI`：axis_id、state_bits、fault_code、angle、velocity、adopted_target_angle、adopted_target_velocity、output_fraction、temperature_c、feedback_age_us。

axis_id：0小Y、1P、2大Y。state_bits：bit0在线、1使能、2回零完成、3角度有效、4速度有效、5采用目标有效、6温度有效、7驱动输出有效。fault_code：0无、1反馈超时、2过温、3过流、4堵转、5启动位置越界、6回零超时、7限位、8驱动器故障，厂商扩展使用0x8000以上并在交付说明列出。

angle 为经过解环的连续关节角，零位和正方向由安装表固定，不得每次启动随意归零。feedback_age_us 指本次快照时距最新电机反馈的时间。output_fraction 是电控定义的归一化驱动输出 [-1,1]，标度须在交付说明明确；没有该数据时填0并清bit7。温度无传感器时清除有效位。

## 7. 姿态、安装关系和坐标系

### 7.1 POSE：68 字节

准确格式 `<BBHIQQIffffffffff`，按顺序：

`sensor_id:u8, frame_id:u8, valid_flags:u16, source_counter:u32, sample_time_us:u64, device_time_us:u64, frame_epoch:u32, q_w:f32, q_x:f32, q_y:f32, q_z:f32, gyro_x:f32, gyro_y:f32, gyro_z:f32, accel_x:f32, accel_y:f32, accel_z:f32`。

sensor_id=0：经过安装转换/关节合成的相机支架姿态；1：小Y原始IMU（当前 UART3）；2：大Y原始IMU（当前 UART5）。编号定义的是物理角色，不能因为内部换 UART 就交换编号。frame_id=0 为支架 G，1 为小Y IMU原生坐标 S，2 为大Y IMU原生坐标 L；本版只允许对应的 0/0、1/1、2/2 配对。

valid_flags：bit0四元数有效、1角速度有效、2加速度有效、3采样时间已映射到MCU、4device_time有效、5时间仅为MCU接收到IMU帧的时间。bits3、5不得同时置位；交车最低允许bit5，但必须说明串口传输/解算延迟。source_counter 每个真实新样本递增；重发旧样本不能递增。device_time 无效时填0。

统一用右手系：支架 G 的 X 朝前、Y 朝左、Z 朝上；世界 W 的 Z 朝上，初始水平朝向定义 X。四元数顺序 **w,x,y,z**，含义为 `v_W = R(q_W_B) v_B`，即把 frame_id 指定的体坐标 B 转到 W。角速度和比力在 B 中表达，单位分别 rad/s 和 m/s²。加速度是传感器比力，不是已经扣除重力的线加速度。

光学相机 C 常用 X右、Y下、Z前，不能直接当作 G。视觉标定和保存 `T_G_C`；电控提供 IMU 与转轴安装关系。所有相关传感器的 W 必须经过对齐；两个 IMU 各自上电得到的航向不能直接相减。

frame_epoch 是坐标系连续性编号：IMU重置、航向重置、零位重置等破坏原坐标连续性的操作必须改变它；同一启动期间不能复用。变化时撤销旧目标并返回非远程控制状态，PC 重新对齐后才能请求控制。geometry_rev 是持久的安装/轴定义版本，改装后更新并要求视觉核验或重新标定。

**小Y IMU如果不随P一起运动，电控必须利用关节反馈和已知安装关系合成 sensor_id=0 的完整 G 姿态。** 也可在定稿时共同选择由视觉合成，但那将修改基础能力约定，且电控必须提供足够的同步原始姿态、编码器和安装变换。本提案默认由电控输出 G 姿态，不能拿小Y原始四元数冒充相机完整姿态。

交车验收：只动小Y时支架yaw对应变化；只动P时支架pitch对应变化；方向和幅值符合实际；回中可复现；大Y运动时世界姿态仍正确。不以“有四元数字段”作为通过条件。

## 8. 控制指令及状态转换

### 8.1 MODE_REQUEST

`mode:u8, axis_mask:u8, reserved:u16, frame_epoch:u32`。PC 只能请求 SAFE、VISION、CALIBRATION，不能请求成为遥控器或强行清除 FAULT。进入控制模式需电控侧物理许可、对应轴在线并完成回零、反馈有效；ACK成功后仍以 STATUS 的 owner/permit 为准。

CALIBRATION 使用电控限定的低速/小范围，发射一律禁止；VISION 使用电控已验证的正常速度/加速度上限。启动寻位、标定和正常跟随采用不同的控制参数，不能把启动限速当成正常跟随的性能上限。

模式切换本身不生成运动目标；在第一条合法目标到达前保持本地安全姿态。没有使能/许可的轴不因收到AIM而自动使能。MODE_REQUEST请求的轴集合必须包含对应控制策略需要的全部轴，否则返回NOT_READY。

### 8.2 AIM_SETPOINT：48 字节

`frame_epoch:u32, produced_mcu_time_us:u64, valid_until_mcu_time_us:u64, reference_frame:u8, axis_policy:u8, valid_fields:u16, yaw:f32, pitch:f32, yaw_rate:f32, pitch_rate:f32, yaw_accel:f32, pitch_accel:f32`。

- reference_frame=0：世界 W 中的视线角；1：底盘 B 中的视线角（选配，需在交付支持表注明）；2：小Y/P关节目标角，只在 CALIBRATION 使用。
- 世界/底盘视线定义 `d=[cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch)]`，yaw正向左转，pitch正向抬头。yaw 使用连续角，跨±π不突然跳转。关节正方向须按安装表转换为同样的人机含义。
- axis_policy=0：小Y+P，大Y由本地策略保持；1：大小Y协调，需caps.bit13。底层轴分配由电控实现，并反馈各轴实际/采用目标；不能静默把协调模式当成仅小Y。
- valid_fields bit0目标有效、bit1速度前馈有效、bit2加速度前馈有效；不支持的有效前馈必须返回 UNSUPPORTED。其余字段填0。不设置bit0表示撤销当前视觉瞄准，进入本地保持并取消尚未执行的发射；它不是“继续上一个目标”。
- 世界/底盘模式中 rate/accel 是对应视线角的时间导数；不是IMU的xyz角速度。
- 默认要求 `produced <= now+5ms`、`now-produced <=50ms`、`produced < valid_until <= produced+100ms`、`now < valid_until`。上位机建议有效期80ms。时间比较必须用有符号差，避免 unsigned 下溢。
- 控制循环只采用最新合法目标，不按队列重放旧目标。若无新目标，deadline到期时先停止发射，并按电控本地已验证策略减速/保持；解除过期状态需要新的合法目标，失联会话还需重新握手。
- ACK表示接受；STATUS.applied_seq 与 JOINT_STATE.adopted_target 表示真正采用。采用速度/加速度限制后应报告约束位。

### 8.3 STOP_REMOTE

当前会话任何模式可请求，优先于一般目标；取消远程目标、待发射请求和底盘远程运动，撤销视觉控制许可。它不能覆盖遥控器人工控制，也不等同于全车断电急停。之后重新进入视觉/标定必须重新 MODE_REQUEST。物理急停和电机故障由电控独立处理。

### 8.4 SERVICE_REQUEST：12 字节

`operation_id:u32, action:u8, axis_mask:u8, reserved:u16, expected_frame_epoch:u32`。operation_id从1递增，需ACK；同ID不得二次启动动作。action=0清除锁存故障、1请求电控已有的回零流程、2取消回零。

清故障仅在底层故障原因已经消失、无运动且有电控侧物理许可时执行，允许当前处于FAULT。不能通过清故障绕过未解决的启动位置越界、限位或过流。回零仅在SAFE且物理允许时执行，使用电控验证过的轴顺序、范围和速度；没有可靠回零流程的轴返回UNSUPPORTED，交付时说明人工恢复方法。取消回零进入电控定义的安全保持状态，不突然切断承重轴输出。

成功ACK表示操作被接受，完成结果看SERVICE_STATE。回零引起坐标零点变化时必须更新frame_epoch，取消所有旧目标。清故障或回零完成后保持SAFE，不自动恢复视觉运动。STOP_REMOTE也取消由远程发起、尚未完成的回零；遥控本地回零不受远程抢占。

SERVICE_STATE：`operation_id:u32, action:u8, state:u8, reason:u16, sample_time_us:u64, frame_epoch:u32, axis_mask:u8, reserved:byte[3]`，24字节。state=0空闲、1接受、2进行中、3成功、4失败、5取消；reason为通用结果码，具体轴故障见JOINT_STATE。空闲operation_id=0。始终保留本会话最后操作结果并周期上报，默认2Hz，状态变化立即上报，避免只发一次完成事件而丢失结果。

## 9. 选配：发射与底盘

### 9.1 发射

SHOOT_REQUEST：`request_id:u32, action:u8, reserved:u8, flags:u16, related_aim_seq:u32, deadline_mcu_us:u64`。action=0取消所有尚未执行的视觉发射，1请求单次发射；flags暂为0。request_id 在会话内从1严格递增，不回绕。重发同一个 ID 不得再发一次；电控保留最后ID及结果，旧ID返回STALE，同ID不同内容返回CONFLICT。若会话内ID耗尽，退出控制后重新握手。

仅 VISION 且物理允许、云台/发射机构就绪、未超热/故障、关联 AIM 已采用且仍有效时才执行；deadline 必须未过期且不超过 now+100ms，也不能超过关联 AIM 的有效期。若关联目标已被新目标替换而尚未开始执行，取消该请求并上报。每次最多一个待执行请求，不积压弹丸请求；BUSY由上位机处理。取消不能撤回已经发生的机械动作。

SHOOT_STATE：`sample_time_us:u64, valid_flags:u32, last_request_id:u32, wheel0_rpm:f32, wheel1_rpm:f32, feeder_rpm:f32, bullet_speed_mps:f32, bullet_speed_age_us:u32, heat:f32, heat_limit:f32, actual_projectile_count:u32, ammo_remaining:u16, reserved:u16`。

valid_flags bit0/1两摩擦轮转速有效，2拨弹转速有效，3实测弹速有效，4热量/上限有效，5真实出膛计数有效，6剩余弹量有效。没有测速传感器就清bit3，视觉继续用自己的估算参数；不能把拨弹转速或配置弹速放在实测弹速里。弹速对应最近一次真实测量，age 表示其年龄。不能把拨弹次数冒充出膛数。

SHOT_EVENT：`request_id:u32, event:u8, evidence:u8, reason:u16, sample_time_us:u64, adopted_aim_seq:u32`。event=0接受、1拨弹开始、2拨弹结束、3检测到出膛、4拒绝、5失败、6取消；evidence=0软件状态、1电机反馈、2出膛传感器。reason 使用通用错误码；event=3仅允许 evidence=2 且caps.bit9。事件不替代遥测，掉线后不能根据软件开始事件推断真实出膛。

### 9.2 底盘（caps.bit12）

CHASSIS_SETPOINT：`produced_mcu_us:u64, deadline_mcu_us:u64, frame_epoch:u32, frame:u8, valid:u8, reserved:u16, vx:f32, vy:f32, wz:f32`，36字节。frame本版仅0=底盘当前体坐标，X前、Y左，wz绕Z逆时针；valid=0撤销，1有效。使用与AIM相同的新鲜度检查，仅VISION且电控物理许可才执行，失效减速停止。限速及加速度是电控参数。其他frame返回UNSUPPORTED。

CHASSIS_STATE：`sample_time_us:u64, odom_epoch:u32, flags:u32, x:f32, y:f32, yaw:f32, vx:f32, vy:f32, wz:f32`。位置/航向在固定里程计坐标 O，速度在当前底盘坐标 B；flags bit0位姿有效、1速度有效、2时间有效、3受到限幅。odom_epoch重置表示里程计不连续，不等于IMU世界系的frame_epoch。导航所需定位地图/外部定位融合不由此协议包办。

底盘指令请求ACK时必须回复；本版STATUS仅报告AIM执行序号，底盘闭环依靠独立STATE反馈，若要求逐指令执行追踪应在定稿时增加专用执行状态消息。

## 10. 参数接口：后续少改固件的关键

只暴露电控审核过的参数；接口必须支持发现字段、单位、范围、当前值和写入条件。不能让视觉猜内存地址或靠更改源代码调值。

### 10.1 枚举和读取

- PARAM_LIST_REQ 0x0300 `<HH`：start_key、limit（1～64）。
- PARAM_LIST_REPLY 0x8300：`start_key:u16,next_key:u16,count:u16,reserved:u16,keys:u16[count]`；返回升序key，next_key=0xFFFF表示结束，否则是下页起点。保留0xFFFF，不能用作参数key。
- PARAM_DESC_REQ 0x0301 `<HH`：key、reserved。
- PARAM_DESC_REPLY 0x8301：`key:u16,type:u8,access:u8,unit:u8,group:u8,conditions:u16,min:byte[8],max:byte[8],default:byte[8],current:byte[8],name:char[32]`，72字节。

type=1 bool（低1字节0/1）、2 u32、3 i32、4 f32；slot均为8字节，小端有效值放低位，其余字节清0。name为ASCII机器名，零终止并补0，最长31字符。access bit0可读、bit1可写RAM、bit2可持久保存。conditions bit0仅SAFE/本地保持且无运动时、bit1仅CALIBRATION、bit2需要轴失能；多个置位是同时满足，因此互斥条件不能同时设置。未列出的参数返回UNSUPPORTED。

unit=0无量纲、1rad、2rad/s、3rad/s²、4ms、5Hz、6rpm、7m/s、8m/s²、9控制器自定义增益、10µs；group=0系统、1小Y、2P、3大Y、4发射、5底盘。自定义增益的算法、量纲和缩放由电控交付说明写明，不能假定不同电机Kp可直接比较。

### 10.2 成组修改和持久化

- PARAM_STAGE 0x0302：`transaction_id:u32,expected_revision:u32,count:u16,reserved:u16`，后跟count组 `key:u16,type:u8,reserved:u8,value:byte[8]`，count=1～16。整组检查类型、范围、权限、模式及交叉约束；任一失败则整组不暂存。
- PARAM_COMMIT 0x0303 `<IIB3s`：transaction_id、expected_revision、persist（0仅RAM/1同时保存）、reserved。
- PARAM_ABORT 0x0304 `<I`：transaction_id。

同一会话最多一个暂存事务，2s不提交自动撤销；transaction_id从1递增。相同事务同内容重试必须幂等；ID相同但内容不同返回CONFLICT；已完成的旧事务不能再次应用。至少保留最近完成事务的结果，并拒绝更旧ID。PC只有上一事务结果明确后才启动下一事务。

提交时重新检查权限、条件及 expected_revision，参数以一组在控制循环边界原子应用，param_rev递增。参数版本在每个boot_id内不得倒退或复用；PC发现boot_id变更后必须重新读取，不能拿上次启动的revision直接写入。

持久化仅在安全空闲条件执行，采用电控实现的掉电一致性方案；persist=1时先完成可靠保存，再原子应用运行值并ACK成功。写入失败保留旧运行值与旧持久值，返回HARDWARE_FAULT。存储任务不能阻塞电机实时控制；保存之间至少间隔10s，频繁请求返回BUSY。普通调参先写RAM，试验确认后主动保存一次；不能每帧写Flash。PARAM_DESC读回必须是实际当前值。

机械硬限位、驱动器保护上限、传感器安装方向等默认只读。需要改变安装/零位定义时走电控批准的操作并更新geometry_rev/frame_epoch，不能通过普通PID调参绕开。

### 10.3 建议交车时实现的参数表

各轴基址：小Y=0x1000，P=0x1100，大Y=0x1200。以下offset加基址生成key。数值范围由电控实测填写在描述中，本文不虚构车辆安全限值。

| offset | name后缀 | 类型/单位 | 权限与语义 |
|---:|---|---|---|
| 0x00 | hard_angle_min | f32/rad | 只读，允许位置下界 |
| 0x01 | hard_angle_max | f32/rad | 只读，允许位置上界 |
| 0x02 | startup_speed_max | f32/rad/s | 只读，启动速度上限 |
| 0x03 | follow_speed_max | f32/rad/s | RAM+保存，有上界；空闲时改 |
| 0x04 | follow_accel_max | f32/rad/s² | RAM+保存，有上界；空闲时改 |
| 0x05 | follow_error_trip | f32/rad | 只读，跟随误差保护阈值 |
| 0x06 | target_deadband | f32/rad | RAM+保存；空闲时改 |
| 0x07 | position_kp | f32/增益 | 电控验证范围内RAM+保存；空闲时改 |
| 0x08 | position_ki | f32/增益 | 控制器存在此项才提供 |
| 0x09 | position_kd | f32/增益 | 控制器存在此项才提供 |
| 0x0A | calibration_speed_max | f32/rad/s | 电控硬上界内RAM+保存；空闲时改 |
| 0x0B | calibration_angle_min | f32/rad | 硬限位内RAM+保存；空闲时改 |
| 0x0C | calibration_angle_max | f32/rad | 硬限位内RAM+保存；空闲时改 |
| 0x0D | startup_hold_angle | f32/rad | 仅经电控验证允许调整时开放；仅空闲、保存后下次启动采用，不改机械零位 |

系统只读参数：0x0001 control_period_us(u32/µs)、0x0002 heartbeat_timeout_ms(u32/ms，500)、0x0003 aim_max_lifetime_ms(u32/ms，100)、0x0004 aim_max_age_ms(u32/ms，50)。参数完整name使用 `sy_`、`p_`、`by_` 前缀。

发射选配参数：0x2000 wheel0_target_rpm、0x2001 wheel1_target_rpm（f32/rpm），0x2002 fire_min_interval_ms（u32/ms）；均只在空闲/停止发射时写入，范围和保护由电控限定。读取当前目标和转速容差的只读诊断参数由电控列入参数清单。

底盘选配参数：0x3000 vx_max、0x3001 vy_max（f32/m/s）、0x3002 wz_max（f32/rad/s）、0x3003 linear_accel_max（f32/m/s²），空闲时写入，不能超过只读硬上界。

无法调整的控制器参数要在交车前明确；“支持参数接口，但所有影响跟随性能的参数都只读”不能满足本次独立调车目标。视觉算法参数仍在小电脑配置文件里，不需要塞进主控参数表。

## 11. 遥测订阅与诊断

SUBSCRIBE：`count:u8,reserved:byte[3]`，后跟count组 `<HHI`：msg_type、rate_hz、source_mask；count=1～16。rate=0取消，1～200为请求频率。POSE source_mask bit0支架、bit1小Y、bit2大Y；其他消息source_mask必须为0。每个订阅元组定义该消息/来源的频率，一条POSE来源发一帧。

整组接受或整组拒绝，不静默降频。重连恢复默认订阅；源数据不足请求频率时不造新样本，可低于请求速率并保持真实source_counter/time。STATUS最低10Hz，不能关闭；DIAGNOSTICS、SERVICE_STATE至少1Hz；订阅变更不能突破带宽预算。SHOT_EVENT按事件产生，不按rate订阅。

DIAGNOSTICS 10个u32：`rx_crc_errors,rx_format_errors,rx_bad_session,expired_commands,duplicate_commands,tx_dropped_frames,tx_max_queue_delay_us,control_period_us,control_max_jitter_us,control_overrun_count`。计数从本次启动累计、允许u32回绕；两个max为最近1s窗口最大值，control_period为标称周期。允许日志结合POSE源计数、时间和JOINT反馈年龄算实际更新率。

## 12. 接入流程

1. PC打开串口，发HELLO并验证版本、能力、boot_id、安装结构版本。
2. 时间同步，接收状态和姿态；必要时配置订阅，读取参数和范围。
3. 验证frame_epoch、姿态有效性和模式；物理许可满足后请求CALIBRATION或VISION。
4. 等STATUS确认owner/permit，再发送带期限的目标，并持续心跳。
5. 调参：读当前revision → STAGE → COMMIT到RAM →读回 →观察实际响应 →确有需要时再事务保存。
6. 结束试验发STOP_REMOTE；断线/MCU重启后从第1步开始，禁止自动补发断线前发射请求。

## 13. 交车验收表

| 项目 | 通过标准 |
|---|---|
| 编码一致 | 双方通过随附CRC和黄金报文；每种固定消息长度一致 |
| 分包/粘包/错误 | 任意拆包、连续多帧、插入噪声、错CRC、错长度可恢复；错误指令不会驱动电机 |
| 稳定通信 | 默认订阅连续10分钟，无解析失步；统计丢包、延迟和控制超时，约定阈值后记录结果 |
| 姿态真实性 | 小Y、P分别运动时G姿态均正确；无新IMU数据时序号/时间不伪更新；断开传感器后及时清有效位 |
| 坐标和零位 | 三轴方向、连续yaw、P正方向、q顺序、安装变换由双方实际确认并签入安装表 |
| 控制反馈 | 同时记录目标seq、采用seq、实际角度、速度和限幅位；能区分收帧、接受、采用和到位 |
| 断线/旧包 | 拔USB、停止目标但保持心跳、重放旧帧、错误session/epoch、MCU重启均不能恢复旧动作 |
| 控制权 | 遥控接管立即生效；CALIBRATION不能发射；没有物理许可时MODE请求有明确拒绝 |
| 参数事务 | 非法整组不生效；合法整组读回一致；revision冲突不覆盖；重复提交不重复执行 |
| 参数掉电 | 保存后重启保留；写入中掉电只恢复完整旧版或完整新版，无半组值；不阻塞控制环 |
| 故障恢复/回零 | 原因未消除不能清故障；操作可查进度、失败原因和最终结果；重复请求不重启回零；完成后不会自行恢复运动 |
| 过期与保持 | 目标过期后各轴行为符合电控确认的保持/减速方案，特别验证P不因普通串口超时掉落 |
| 选配发射 | 重发同一request_id不重复动作；过期/旧AIM被拒绝；拨弹事件与真实出膛证据区分 |
| 正常跟随性能 | 电控给出阶跃/扫频或连续轨迹实测记录：延迟、可跟随速度/加速度、误差和饱和；区分启动/标定/正常参数 |

通信和数学部分可先离线验收；带电机/发射机构的项目由现场人员按电控确认的安全工况测试。验收记录中注明固件构建号、参数版本和geometry_rev。

## 14. 当前工程迁移说明

目前的工程仍存在两条不同数据链：正式视觉主链路使用旧 SP 控制/反馈；此前相机标定使用 HI91 四元数转发。当前代码不能直接切换到本提案而自动工作。

实施顺序：双方确认本文 → 电控在正式串口实现基础消息与参数表 → 用离线样例联调 → 视觉实现RMV通信类、姿态历史插值、状态/回执和调参入口 → 用同一安装表做双轴姿态验收 → 接入标定和正式视觉程序。

旧SP里复用弹速/弹量字段传转速或状态的做法在RMV里停止；旧字段解析只保留在明确选择的旧协议适配中。正式运行一次只选一种协议，不在同一字节流里猜测和混用。

**双方交车确认的核心句：基础能力全部实现、参数范围和安装关系填写完毕、验收表通过，视觉才能独立完成约定范围内的后续调试。**

## 15. 参考程序与验证范围

项目内参考实现为`tools/rmv/protocol.py`。使用Python 3.10及以上，无第三方依赖，在工程根目录执行：

```text
python -m tools.rmv.protocol --self-test
python -m tools.rmv vectors logs/rmv/serial_test_vectors.json
```

参考程序覆盖帧头、CRC、固定负载布局、流式拆包和错误恢复；JSON包含各消息的payload与完整帧十六进制。参数示例范围仅供验证序列化，不是实车允许范围。样例各自独立，不构成可直接回放的控制会话。

离线自测不能替代主控状态机、权限、幂等、时间检查、掉电保存和真实电机测试。参考解析器只输出经过帧级校验的数据；实际应用必须继续检查会话、准确消息长度、保留位、数值有效性、物理许可和业务条件后才允许执行。

## 大Y/P私有跟踪扩展（始于2026100604）

这段仅描述临时 RMV 固件，不替代电控后续正式固定帧。capabilities bit13 已开启时，MODE_REQUEST 的 mode=3、axis_mask=6 选择大 Y/P；AIM_SETPOINT 保持48字节，reference_frame=2、axis_policy=2 表示 yaw_target 为大 Y 的原电机圆周角（rad，正向左转），pitch_target 仍为原 P 关节角（rad，正向抬头），小 Y 由下位机保持接管时的位置。axis_policy=0 与 axis_mask=3 仍是原小 Y/P 路径。两种组合不得混用，旧固件不接受私有 policy=2。

bit13 在当前私有固件仅表示该大 Y/P 关节扩展；不证明大 Y IMU 世界参考已对齐，世界目标 capability bit2 和发射能力仍关闭。JOINT_STATE 的 axis_id=0/1/2 继续分别表示小 Y/P/大 Y，含实际角速度、目标及采用序号。目标有效期限60 ms；失效或撤权后大 Y 速度归零并进入本地保持。机械参数区、标定原始姿态 profile 保持兼容。

0603在0602相同帧格式/能力位基础上，将私有policy2大Y/P跟踪最高速度从0.1/0.08提高到0.2/0.16rad/s；加速度、增益、P范围和目标到期不改。P私有跟踪速度独立于原0x110A标定速度，原标定模式和持久参数数量/存储格式不变。

0604沿用原ZIP control.cpp的大Y自瞄速度上限2.0rad/s及P位置速度接口1.5rad/s；私有policy2取消额外大Y速度斜坡和P Slew轨迹。大Y比例环Kp5、P原机械范围、数据新鲜度及目标期限保持。P采用目标直接报告实际下发位置，target_velocity填0，实际速度仍在velocity字段。原标定模式及参数存储格式不改。


### 当前私有关节VISION单发扩展（0605，已烧录）

保留原RMV消息长度。caps.bit7开启时，MODE_REQUEST mode=2/axis_mask=6允许 reference_frame=2/axis_policy=2 的大Y/P关节控制；这是当前固件私有扩展，不代表世界系瞄准能力开启。MODE3仍拒绝SHOOT_REQUEST。普通0604跟踪不需要此扩展。

SHOOT_REQUEST接受后最多保存一个待执行单发，关联AIM采用前不启动；新AIM替换、无效目标、期限或物理许可撤销会取消。实际执行仍需关联AIM、有效关节/姿态、P发射包络、摩擦轮/拨弹就绪及大Y/P关节误差<=0.03/0.02rad。ID在会话内递增，同ID同内容返回原结果，同ID不同内容CONFLICT；BUSY/拒绝不会在重试时转为执行。没有自动发射重试。

SHOOT_STATE默认20Hz，仅bits0/1/2声明实际两摩擦轮和拨弹rpm。没有测速/出膛、热量、余弹证据，bits3–6清零；弹速0与age=UINT32_MAX表示未知。SHOT_EVENT使用接受/拒绝/取消/拨弹开始/失败（软件证据）及拨弹结束（编码器与电机状态证据）；不发送“检测到出膛”事件。STATUS.fire_permit表示此刻独立遥控触发和机构就绪，不等于识别到可击打目标。独立遥控许可：双上AUTOAIM且左摇杆向上过半，松杆停止发射。


### 2026100701私有关节轨迹执行

当前私有capability bit15表示policy2/mask6大Y+P关节轨迹，实际capabilities=44187；bit14触发事件能力仍未定义、不声明。AIM帧长度保持48字节，reference=2/policy=2/valid_fields=7，六量依次为大Y/P关节角度、角速度、角加速度，rad/rad/s/rad/s²；produced是六量共同的时间原点。0605及更早版本拒绝有效前馈；0701只为该私有策略接受完整六量，标定/其他策略仍拒绝。非有限量、速度超过原2/1.5rad/s、加速度绝对值超过现有MPC100rad/s²、失配/过期拒绝。

控制周期按时间原点展开二次轨迹，积分在原速度上限饱和后按饱和速度继续，不通过限速饱和产生位置跳变。P范围仍为[-0.72,0.45]rad，边界向外的轨迹速度清零并报告约束。大Y采用轨迹瞬时速度+Kp5圆周位置误差，限幅2rad/s；P沿用原PositionSpeed位置接口和1.5rad/s移动上限，目标速度遥测为已采用位置轨迹的导数。小Y保持、遥控权限、目标期限、故障/新鲜度、标定、参数存储格式及单发幂等性保持。SHOOT_REQUEST关联当次完整轨迹AIM，MCU按执行时刻展开的角度核对关节误差。
