# 配置与标定状态

2026-10-06 当前 RMV 调试使用固件2026100602。`configs/standard.yaml` 中 `rmv_debug_yaw_axis: big`，`rmv_debug_max_displacement_deg: 0.0`：大 Y/P 跟踪、小 Y 保持、无相对角度限幅；默认只读，显式 `--rmv-follow` 才允许跟踪，P 原机械范围保留。安装候选、光心/枪口与真实枪管方向继续使用。标定配置未改，完整标定放行仍false。操作见 [当前RMV调试](rmv_debug.md)，下文历史状态以日期区分。

正式运行配置为configs/standard.yaml，标定工具配置为configs/calibration.yaml。启动器生成的logs/xuc-bringup/field_autoaim.runtime.yaml是缓存；不要将它作为另一份配置源。颜色选择保存在~/.config/sp-vision/field_autoaim_enemy_color。

## 2026-10-05 当前核对

当前小电脑工程为 `/home/rcsdart/rm27`，主控版本 2026100501，通过 UART4 460800 发送 RMV。相机当前以 1440×1080、150 FPS 工作；与串口联合采集通过，三轴反馈在线，详情见 [USB 与相机诊断](usb_camera_diagnosis.md)。双中为固件 RESET 档，会使能电机，不能作为关闭档。

用户确认此前标定后未更换镜头或调整对焦。小电脑的正式配置与标定配置内参、畸变系数一致，数值有限、焦距为正、主点位于图像内；分辨率与当前图像一致。因此暂未发现需要仅因更换小电脑而重标内参的依据。这只是配置一致性核对，不是重新验证实际标定精度。

当前 `rm27` 未找到 `data/calibration/sentry_intrinsics` 的原始标定图或 `logs/calibration-quality.json`；下文的 29 张图和重投影误差是此前记录，当前无法据原图复核。检查结果保存于 `logs/usb-diagnosis/calibration-audit.json`。

`capture` 已新增 RMV 标定接收器，当前 `calibration_pose_source: rmv` 接收 `sensor_id=1/frame_id=1` 的小 Y 原始 IMU 四元数；旧 HI91 接收仍可通过 `hipnuc` 选择。RMV 只发送 HELLO、空闲心跳和 TIME_SYNC，不请求控制、发射或参数写入。安装轴方向和实测变换仍未确认，`calibration_pose_verified=false` 保留；内外参及固件确认标志未更改。2026-10-05 已修复 HELLO 缺少 ACK_REQUEST 的握手问题；小电脑重新编译后，3 秒实车姿态诊断接受 227 个样本，fresh=true、stationary=true、错误为空。此结果验证静止通信及姿态读取，安装轴方向和运动关系仍需现场验收。

用户于 2026-10-05 明确确认：UART3 的小 Y IMU 与相机相对位置恒定，两者刚性固定并随 P 轴运动；IMU 大致位于相机光心上方 2 cm。此项确认解决了是否同随 P 运动的安装位置疑问。约 2 cm 是现场位置估计，尚未给出精确测量、传感器测量原点、完整三维偏移或坐标轴定义；不能由此确定安装旋转。安装轴方向、运动响应及大 Y IMU 坐标对齐仍未完成验证。固件确认标志保留现值。

以下姿态直通、P 轴保护和外参流程描述为 2026-09-17 的历史记录，不是当前 RMV 固件的即时状态。

## 2026-10-05 原始 RMV 相机/IMU 旋转标定

窗口修复验收完成：C++ capture 的 GTK3 合成图像窗口在实际桌面 DISPLAY=:0 创建/关闭成功；部署检查 software_ready=true、capture_gui.ready=true。capture 的所有 OpenCV 依赖均来自系统库、无 /usr/local 混链。系统 OpenCV 下 27 项离线 CTest、采集入口检查及 RMV 私有 PTY 有效场景通过。未由代理打开真实相机或保存图像，目标手眼目录当前无图像；用户可直接重跑原采集命令。证据见 logs/deployment/opencv-gui-verification.json。

2026-10-05 21:10:48 首次 GUI 采集退出原因已定位：C++ capture 链接 /usr/local 的无 GUI OpenCV 4.5.4；系统 /usr 的 OpenCV 4.5.4 已带 GTK3。相机此前已经正常产生 1440×1080 图像，该错误来自 cvShowImage。现有 build 统一改用 /usr/lib/x86_64-linux-gnu/cmake/opencv4，scripts/build.sh 默认优先系统 OpenCV（SP_OPENCV_DIR 可显式覆盖）。capture 增加 --gui-info（无窗口/无设备）、--gui-check（合成图像窗口，无相机/串口）及正式采集前的窗口检查。check_deployment.py 先检查 --help 是否支持 gui-info，再执行设备无关检查；不会把新选项传给旧采集器后误开设备。

P、小 Y 独立运动和 IMU 随动已核对，用户确认 IMU 与相机刚性固定。原始传感器采集使用 `calibration_raw_pose_verified=true`；`calibration_pose_verified=false` 仍表示相机支架 G 安装变换尚未完成验证。采集接收 sensor1/frame1 原始四元数，保留停稳、时间差、来源和 boot/epoch/geometry 检查，不套旧 `R_gimbal2imubody`。

在小电脑本地终端由用户启动，固定当前配置对应的棋盘（11×8 内角点，每格 15 mm），保持棋盘不动，只手动转相机：

```bash
cd /home/rcsdart/rm27
source scripts/runtime_env.sh
./build/capture configs/calibration.yaml -o=data/calibration/sentry_handeye
```

遥控左拨杆中、右拨杆下：左摇杆左右控制小 Y；右摇杆上下控制 P，左右控制大 Y。采集时大 Y 保持不动。小幅改变小 Y 和 P，包含两轴单独变化及组合角度，共保存 20–30 组，至少 15 组有效图像。每次松杆、等待约 1 秒、确认完整棋盘被检测且姿态有效后按 `s`；按 `q` 退出。所有图像来自一次启动/epoch/geometry；重启或重置后不得混入旧数据。首轮窗口失败仅创建了数据集标记、未保存图像，可继续使用同一目标目录。

```bash
python3 scripts/calibrate_rmv_rotation.py data/calibration/sentry_handeye
```

此工具用原始传感器到导航系旋转与棋盘 PnP 旋转求解 `R_camera2imubody`；不使用旧 G 安装矩阵，不假定 IMU 原点位置固定，不估计平移。结果为 `logs/calibration/rmv-rotation.json` 的旋转候选，含来源哈希、运动可观测性及每四张留一张的独立验证。训练及留出 RMS 阈值均为 0.5°、最大误差阈值 1.5°；质量合格也不自动写 YAML 或固件。棋盘角点编号翻转不自动修正，若造成不一致会使质量检查失败。只有单轴、无有效时间/姿态元数据或混 boot/epoch 的数据会拒绝。旧 `calibrate_handeye`、`calibrate_robotworld_handeye`、`verify_handeye` 明确拒绝原始 RMV 数据。

迁移中已通过 27 项离线 CTest、6 个原始旋转测试（含 25 张合成棋盘图像的检测/PnP/CLI）、5 个 HI91/20 个 RMV 私有 PTY 场景及采集入口检查。重新编译的采集器只读实车检查接受 232 个样本，fresh=true、stationary=true、版本仍为 2026100501；没有启动相机 GUI 或电机测试。

求出旋转后仍需核对 G 轴定义、关节零位、原点/平移和导航系 Z 向上含义；相机到 IMU 的原始旋转不能直接复制成 `q_sensor_from_gimbal` 或 `R_camera2gimbal`。

## 当前 RMV 姿态检查与采集

2026-10-05 20:45:44 的 30 秒记录及用户确认完成小 Y 独立遥控核对：用户先左推、松杆保持，再右推、松杆保持。小 Y 关节摆幅 13.931°，P/大 Y 仅波动 0.022°/0.049°；原始小 Y IMU 的拟合轴投影摆幅 13.630°，比例 0.9792、相关系数 0.999984、旋转向量拟合 RMS 0.042°。最后 20–29.5 秒小 Y 关节波动 0.044°。RMV 小 Y 关节负值方向为左转、正值方向为右转，与大 Y 关节正值左转相反。完整接收帧无 CRC/格式错误，三轴无故障；MCU 记录 1 次遥测发送丢弃，不声明零丢包。末尾位置比开始偏右约 4.395°，不作为回原位漂移测试。P、大 Y、小 Y 的 IMU 随动均已有记录，但安装矩阵、相机支架 G 姿态及世界目标到关节方向仍未完成统一，calibration_pose_verified 和固件安装放行标志保持 false。详细记录见 logs/serial-probe/20261005_204544_diagnostic/motion-summary.json。

用户现场确认：左摇杆向左推时，小 Y 向左转；左侧输入的实际转向已核对。本次松杆保持、右侧输入及 IMU 随动已由下述记录和用户确认核对，安装及标定确认标志保持 false。

关闭占用同一串口的其他程序后，由用户在小电脑终端运行：

```bash
cd /home/rcsdart/rm27
source scripts/runtime_env.sh
./build/capture configs/calibration.yaml --pose-check=3
```

此命令不打开相机，只进行 RMV 诊断会话，报告接收样本数、原始 wxyz 四元数、源计数、boot/session/epoch、时间质量、`fresh` 和 `stationary`。`fresh=true` 表示当前样本有效且新鲜；`stationary=true` 还要求角速度小于 2°/s、0.5 秒内相对停稳锚点的转角小于 0.5°。检查返回 0 仅证明姿态接收条件，不证明安装轴或外参正确。运动时可保持 fresh，但应为 stationary=false。

2026-10-05 已按用户要求烧录构建 `2026100501`：左拨杆中、右拨杆下（SEPARATE），左摇杆左右独立控制小 Y，右摇杆左右控制大 Y、上下控制 P。小 Y 目标最高变化率约 10.55°/s，松杆保持当前目标；进入该档及反馈恢复时以实际位置建立目标，沿用原机械包络、速度/电流限制，反馈超过 100 ms 或温度超过 70°C 时停止小 Y 输出。未接入大 Y 自动补偿。本次用户小幅测试已确认左右响应和松杆保持，原始 IMU 随动一致。 重启后 3 秒只读诊断确认三轴在线、故障掩码为 0、CRC/格式错误为 0；小 Y 已处于位置控制。另一次姿态检查接受 229 个样本，fresh=true、stationary=true。后续可运行 `python3 scripts/probe_rmv_diagnostics.py --seconds 30`，保持右摇杆居中，由用户只做小 Y 左右各约 5° 的动作并观察松杆保持，以核对方向和 IMU 随动。当前小 Y 四元数保持传感器原始坐标含义，接收器不会套用旧安装矩阵，也不会把它当成 sensor_id=0 的相机支架姿态。实车验证完成前，不更改标定确认标志。

原始 RMV 采集时，采集器要求姿态与图像主机时间相差不超过 50 ms，且图像位于当前至少 0.5 秒的停稳区间；保存前再次检查当前有效性。图像时间来自相机主机时间戳，尚不是验证过的硬件曝光时间。MCU 接收时间质量会明确标出，不能当成 IMU 硬件同步精度。

RMV 样本仍保存 `N.jpg` 和 `N.txt`（原始 wxyz），同时保存 `N.pose.json` 的来源、时间及版本信息。数据集使用 `pose_source.txt` 与 `pose_context.json` 防止混入 HI91 数据或不同 boot/epoch/安装版本的姿态。epoch 或 geometry 变化、会话失效后接收停止，不自动重连；应核对坐标连续性后使用新的采集数据集。

离线验证命令：`python3 tests/calibration_pose_test.py`。该测试仅打开新建的 Linux PTY；覆盖 5 个旧 HI91 场景、20 个 RMV 场景及采集入口门控，不操作实际设备。纯状态机测试已加入离线 CTest；交付 ZIP 存在时也执行真实固件协议代码回归，检查正确 HELLO 与 BAD_VALUE/UNSUPPORTED 拒绝原因。

2026-10-05 20:13:40 的 60 秒 RMV 记录支持 P 随动关系：P 关节摆幅 10.459°，小 Y IMU 在拟合旋转轴上的投影摆幅 10.647°，比例 1.015、相关系数 0.99994；大 Y 对应为 26.664°/26.238°，比例 0.986。旋转轴由同一段数据拟合，仅支持运动一致性，不是独立安装变换标定。小 Y 自身摆幅仅 0.132°，没有得到独立运动验证，标定确认标志保持 false。唯一 CRC 错误候选属于握手前的旧 session=5，本次 session=6 的完整报文无 CRC 错误；原始整段通信失败结果保留。分析见 logs/serial-probe/20261005_201340_diagnostic/motion-summary.json。

早期用户描述第一次 P 为低头、大 Y 为左转，曾被错误对应到某段 P 正向反馈；该“P 正向低头”解释已由 22:11 的明确抬头/低头单轴复核撤销。当前 P 正向抬头、大 Y 正向左转；P 原始旋转轴接近 sensor -Y，与新几何模型 G=base·Rz(-small_Y)·Ry(-P) 一致。SP Aimer 俯仰正值仍为低头，进入 RMV 世界目标须取负；安装与控制放行仍待完整几何核验。

## 内参：已完成

2026-09-17，海康MV-CS016-10UC、1440×1080。DFvision Q12-200-15，12×9方格、11×8内角点；格长15 mm由用户确认。29张图像全部识别，k3固定为0。平均逐点重投影距离0.0894 px，RMS 0.10465 px。
留出6张检查的RMS为0.0580～0.1672 px；每张仍拟合自身板姿态，不等于独立距离精度验证。23张训练与29张全量的fx/fy差分别为-0.0707%/-0.0569%。角点覆盖凸包约60.7%，极边缘未完全覆盖。
结果已写入两个现有配置。原始数据为data/calibration/sentry_intrinsics，详细质量记录为logs/calibration-quality.json。镜头焦距、对焦、分辨率或裁剪改变需重新核对。
内参采集：`./build/capture configs/calibration.yaml --intrinsics-only -o=data/calibration/sentry_intrinsics`；s保存、q退出。只保存图像，不生成姿态文件。
计算：`./build/calibrate_camera data/calibration/sentry_intrinsics -c=configs/calibration.yaml --no-display`。

## 历史姿态：HI91 直通回传

用户确认小Y IMU在UART3，大Y IMU在UART5。现场接收测试确定小Y为921600，大Y为115200。原固件错误地从USART1读小Y、将USART3用于裁判系统，且HI91长度拼接、CRC和字段读取有问题。
当前专用标定固件已修正并烧录：USART3由小Y IMU独占，标定模式禁用冲突的裁判收发；UART4 460800只读转发校验通过的小Y HI91完整帧，保留原始wxyz四元数、设备时间戳与CRC。串口DMA忙时丢弃新发包，不覆盖在途缓冲。没有恢复旧自瞄控制任务。
两路IMU均约100 Hz。小电脑5秒收到500帧、CRC错误0；capture --pose-check=3收到299帧、fresh=true、stationary=true。实际回传通过，但还不能以静止接收代替两轴运动及安装坐标验证。
此前约40秒只读记录中小Y摆幅13.49°、P摆幅10.39°，大Y/UART5的CRC有效姿态不变；这与大Y安装位置相符。SWD在运行中读取DMA缓冲可能跨帧，快照CRC失败不用于估算串口误码率。
协议参考：[HiPNUC官方解码器](https://github.com/hipnuc/products/blob/master/c/hipnuc/hipnuc_dec.c)。传感器原始轴和欧拉角约定不直接等同本车云台坐标，手眼采用原始完整四元数。

## 历史外参流程：尚未完成

当前P轴state=4/fault=3，位置超出保存目标±0.20 rad启动范围，保持失能；小Y遥控未放行。等待用户用支撑物托回原保存角度附近、手离开、摇杆回中后检查。没有扩大启动范围或更改保存目标32685。
恢复后先验证小Y姿态分别响应小Y和P，并确认IMU与相机刚性连接关系、轴方向和R_gimbal2imubody。若IMU只随小Y而不随P，需要正确的机构变换，不能直接套用现有手眼模型。
`calibration_pose_source: hipnuc`选择只读HI91接收；`calibration_pose_verified: false`会阻止外参保存，姿态验证完成后才修改。旧XUC解析保持原样，不与HI91混解。
采集工具支持`--pose-check=3`，仅检查姿态，不打开相机；正常采集要求姿态更新不超过100 ms、角速度小于2°/s且0.5秒内转角小于0.5°。断流、重复时间或未停稳均拒绝保存。数据目录保存来源标记hipnuc_uart3_wxyz，避免混入旧姿态样本。
全部现场角点与姿态确认后，固定棋盘、只转动相机采集data/calibration/sentry_handeye；计算并检查外参后再更新现有配置。当前R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody仍是旧安装值，不能视为哨兵实测。

用户之前提供的机器人中心坐标为X前/Y左/Z上，光心(15,0,50) cm、机身(10,0,50) cm、炮口(10,0,45) cm；这些记录尚未应用，不能替代相机—云台手眼结果。
运行相机工具前沿用现场MVS环境变量（/opt/MVS/lib/64等），不复制SDK。


### RMV 原始姿态采集的停稳判定

2026-10-05 静止串口实测 931 帧：陀螺仪单帧范数中位数 1.18°/s，6.87% 超过 2°/s；原先每个尖峰重置 500 ms 计时，只有 6.77% 的采样时刻可保存。

`capture` 现在使用约 100 ms 窗口（至少覆盖 80 ms）的陀螺仪范数 RMS，小于 2°/s 才判为安静；单帧达到 5°/s 仍立即拒绝。连续姿态相对停稳起点变化须小于 0.5°，停稳计时仍为 500 ms。允许孤立的小噪声尖峰，不允许持续运动。图像选择时间差不超过 50 ms 的最近姿态，检查该图像时刻的停稳状态；处理图像之后的运动不撤销先前已有效的图像。会话、时钟、姿态新鲜度、源计数和坐标版本检查仍生效。

海康相机单帧队列保留最新图像，避免窗口或检测阻塞时滞留旧帧。采集窗口显示拒绝原因、gyro/RMS 和停稳时间；`logs/calibration/capture-state.json` 每秒更新状态，并记录最近一次保存失败原因。图片清晰与图像/姿态同步都需要满足，不能用关闭姿态检查来替代。


### 本次原始旋转结果与时间批次

当前 `data/calibration/sentry_handeye` 含 68 组，原始文件全部保留。1–5.jpg 与 6–68.jpg 之间有 818.54 秒间隔，用户确认期间棋盘格和底座未移动。完整固定棋盘模型验证 RMS 0.552°，不通过；不能忽略这项结果或宣称漂移原因已证实。重新拟合相机内参未改善全量一致性，现有内参保持。

针对最近连续采集批次的可复现命令：

```bash
python3 scripts/calibrate_rmv_rotation.py data/calibration/sentry_handeye --capture-segment=2
```

默认仍使用全量数据。时间批次由图像时间戳间隔大于 120 秒划分，可用 `--max-gap-seconds` 指定；`--capture-segment` 是从 1 开始的显式选择，不按残差自动剔除。输出同时保存全量评估和未选用的图像/哈希。任何批次的坐标来源错误仍拒绝，不能通过选段掩盖混合 boot/epoch/geometry/firmware。

第 2 段的 63 组采用训练 47 组、独立验证 16 组：训练/验证 RMS 0.346°/0.376°，最大误差 0.855°。这只是该连续批次的相机到原始 IMU 旋转候选；跨较长时间的导航系一致性仍需独立核对。平移与 G 安装变换为 null，正式运行 YAML 不据此自动修改。再次固定棋盘，在较短时间内采 15–20 组新的双轴姿态，可独立检查候选旋转与跨批次不一致问题；新数据不得与本次数据混作拟合训练。


独立复测仍保持棋盘和机器人底座不动，采到新的目录并覆盖小 Y 与 P 两个方向，至少 15 组。退出采集后，用固定旋转和固定棋盘参考进行文件验证：

```bash
./build/capture configs/calibration.yaml -o=data/calibration/sentry_handeye_verify
python3 scripts/calibrate_rmv_rotation.py data/calibration/sentry_handeye_verify \
  --verify-candidate logs/calibration/rmv-rotation.json --same-fixed-board \
  -o logs/calibration/rmv-rotation-independent-check.json
```

该入口不重新拟合相机到 IMU 的旋转，`--same-fixed-board` 也不重新拟合棋盘在导航系中的参考方向，从而能检查跨批次一致性。若棋盘或底座移动，应去掉该选项，此时只验证固定安装旋转，不能认证绝对导航参考。旧数据重复用作复测、不同坐标版本/固件或标定配置、覆盖候选结果都明确拒绝。原始数据保持。


本次独立复测的两项结果已分开保存。`rmv-rotation-independent-check.json` 固定安装旋转和原棋盘参考，验证 RMS 0.632°，未通过；`rmv-rotation-independent-mount-check.json` 固定安装旋转，仅估计本批棋盘参考，验证 RMS 0.414°，通过。后者仍未重新拟合安装旋转，来源为真正的新 24 组图像，但不能认证跨批次导航参考。复测批次棋盘参考与原参考相差约 0.446°，主要是导航 Z 航向，偏移原因待核对。无需为安装旋转重复采图。

HiPNUC [官方指令与编程手册](https://download.hipnuc.com/products/imu/cum.html) 定义 WXYZ 为载体系到导航系的旋转，导航系为 ENU；不同载体输出轴向的 Euler 约定不同，6 轴模式存在相对航向漂移。这些资料不能代替当前模块型号、工作模式、URFR/COORD 的读回确认；本次未直连 IMU 配置串口或修改其参数。原始四元数继续直接使用，本轮不增加经验角度补偿，也未修改旧运行外参。


2026-10-05 22:11 独立方向复核确认 P 关节正向抬头、负向低头。仅世界系逆解的几何方向改为 +1，不改变 native/encoder 转换 `pitch_sign=-1`。未烧录构建现为 2026100503，实车仍为 2026100501。三轴人机方向：小 Y 负向左转、正向右转；P 正向抬头；大 Y 正向左转。安装旋转候选在 `logs/calibration/support-axes-candidate.json`。

2026-10-06 用户测得光心相对 P 转轴中心前方 150 mm、上方 50 mm、无左右偏移，并确认测量时镜头水平朝前。按前次指令以前向沿镜头朝向、上向沿物理竖直方向解释，原始测量框架 H 的向量为 `[0.150,0,0.050]` m。当前 G 是关节零位定义的支架系，镜头光轴在 G 内约低 15.6°，因此不能直接把 H 的三分量写成 G 外参。令 `f_G=R_camera2gimbal[:,2]`；在底座水平、P 轴水平的理想模型下，水平镜头姿态的 `P=atan2(-f_G.z,f_G.x)≈+15.594°`，上向为 `u_G=[sin(P),0,cos(P)]`，则 `t_camera2gimbal=0.150*f_G+0.050*u_G≈[0.157917,0.000948,0.007838]` m。约 0.95 mm 的 G 左向分量来自坐标轴方向差异，不是新增的物理左右偏移。变换保持转轴到光心的距离约 158.114 mm，且 `f_G·u_G=0`。

上述底座水平计算是早期假设估计，已由同测量姿态的实际诊断换算替代，不作为当前运行外参。2026-10-06 16:46:38 的5秒记录中 P=11.349°（中位数）、摆幅0.022°；相机光轴相对原始导航系仰角约-2.000°，与用户“水平朝前”的描述存在约2°差异。用户随后确认测量后没有调过 P，因此可将实际记录与原始尺寸关联，不必把 P 的模型差4.245°直接写成零位补偿。

以原始导航 Z 为重力上向，使用 `u_G=R_sensor_from_gimbalᵀ·R_nav_from_sensorᵀ·[0,0,1]`；对386个真实小 Y 姿态的上向逐分量取中位数并归一化。把 `f_G` 投影到该上向的水平面并归一化，得到 `h_G=normalize(f_G-(f_G·u_G)u_G)`，左向为 `u_G×h_G`。物理水平前/左/上测量向量 `[0.150,0,0.050]` m 因而对应 `t_camera2gimbal=0.150*h_G+0.050*u_G≈[0.157550,0.000389,0.013341]` m，转轴到光心距离仍约158.114 mm。静止加速度方向估计的光轴仰角中位数约-2.065°，支持本段导航上向与重力方向一致；不据此认证大 Y/底座的世界安装旋转。

原始尺寸、对应 P、测量后未调 P 的用户确认、姿态来源哈希、转换矩阵和公式均记入现有候选，核验在 `logs/deployment/camera-level-verification.json`。G 原点选在 P 转轴中心；field_verified=false、translation_applied=false，安装旋转/机械零位/世界参考仍需联合验收，尚未写入运行 YAML。不把 IMU 大致高于光心 2 cm 当成转轴外参。

用户随后确认镜头与枪管平行并同随P，按保持同一P位置的测量指令给出枪口前/左/上偏移 `[130,0,0]` mm。用同一个 H→G 变换得到 `t_muzzle2gimbal≈[0.126356,0.000776,-0.030555]` m；枪管方向候选 `b_G=R_camera2gimbal[:,2]`。弹道须从枪口原点计算，不能直接把P转轴或光心当作出膛点；转动后的枪口位置应由完整刚体变换更新。物理平行关系为用户确认，精度未独立测量。

当前G按原关节零位构造，+X尚不等于枪管前向。三轴符号已由实车运动核对，现有native零位保持；若正式控制采用实际关节目标，上位机须在逆解中使用 `b_G`，再按电控最终参考转换。若双方改为枪管前向定义G，则所有安装旋转、位置、关节角及限位需一致重表达，不能只重置P。现有私有RMV `world_to_joints` 的+X假设尚未适配该安装，控制许可关闭；其当前枪管视线保持反例和考虑b_G的972组数学验证在 `logs/deployment/joint-geometry-verification.json`。完整安装交付见 `logs/calibration/installation-handoff.txt`。
