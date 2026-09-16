# 配置与标定状态

正式运行配置为configs/standard.yaml，标定工具配置为configs/calibration.yaml。启动器生成的logs/xuc-bringup/field_autoaim.runtime.yaml是缓存；不要将它作为另一份配置源。颜色选择保存在~/.config/sp-vision/field_autoaim_enemy_color。

## 内参：已完成

2026-09-17，海康MV-CS016-10UC、1440×1080。DFvision Q12-200-15，12×9方格、11×8内角点；格长15 mm由用户确认。29张图像全部识别，k3固定为0。平均逐点重投影距离0.0894 px，RMS 0.10465 px。
留出6张检查的RMS为0.0580～0.1672 px；每张仍拟合自身板姿态，不等于独立距离精度验证。23张训练与29张全量的fx/fy差分别为-0.0707%/-0.0569%。角点覆盖凸包约60.7%，极边缘未完全覆盖。
结果已写入两个现有配置。原始数据为data/calibration/sentry_intrinsics，详细质量记录为logs/calibration-quality.json。镜头焦距、对焦、分辨率或裁剪改变需重新核对。
内参采集：`./build/capture configs/calibration.yaml --intrinsics-only -o=data/calibration/sentry_intrinsics`；s保存、q退出。只保存图像，不生成姿态文件。
计算：`./build/calibrate_camera data/calibration/sentry_intrinsics -c=configs/calibration.yaml --no-display`。

## 姿态：回传已接通，双轴响应待确认

用户确认小Y IMU在UART3，大Y IMU在UART5。现场接收测试确定小Y为921600，大Y为115200。原固件错误地从USART1读小Y、将USART3用于裁判系统，且HI91长度拼接、CRC和字段读取有问题。
当前专用标定固件已修正并烧录：USART3由小Y IMU独占，标定模式禁用冲突的裁判收发；UART4 460800只读转发校验通过的小Y HI91完整帧，保留原始wxyz四元数、设备时间戳与CRC。串口DMA忙时丢弃新发包，不覆盖在途缓冲。没有恢复旧自瞄控制任务。
两路IMU均约100 Hz。小电脑5秒收到500帧、CRC错误0；capture --pose-check=3收到299帧、fresh=true、stationary=true。实际回传通过，但还不能以静止接收代替两轴运动及安装坐标验证。
此前约40秒只读记录中小Y摆幅13.49°、P摆幅10.39°，大Y/UART5的CRC有效姿态不变；这与大Y安装位置相符。SWD在运行中读取DMA缓冲可能跨帧，快照CRC失败不用于估算串口误码率。
协议参考：[HiPNUC官方解码器](https://github.com/hipnuc/products/blob/master/c/hipnuc/hipnuc_dec.c)。传感器原始轴和欧拉角约定不直接等同本车云台坐标，手眼采用原始完整四元数。

## 外参：尚未完成

当前P轴state=4/fault=3，位置超出保存目标±0.20 rad启动范围，保持失能；小Y遥控未放行。等待用户用支撑物托回原保存角度附近、手离开、摇杆回中后检查。没有扩大启动范围或更改保存目标32685。
恢复后先验证小Y姿态分别响应小Y和P，并确认IMU与相机刚性连接关系、轴方向和R_gimbal2imubody。若IMU只随小Y而不随P，需要正确的机构变换，不能直接套用现有手眼模型。
`calibration_pose_source: hipnuc`选择只读HI91接收；`calibration_pose_verified: false`会阻止外参保存，姿态验证完成后才修改。旧XUC解析保持原样，不与HI91混解。
采集工具支持`--pose-check=3`，仅检查姿态，不打开相机；正常采集要求姿态更新不超过100 ms、角速度小于2°/s且0.5秒内转角小于0.5°。断流、重复时间或未停稳均拒绝保存。数据目录保存来源标记hipnuc_uart3_wxyz，避免混入旧姿态样本。
全部现场角点与姿态确认后，固定棋盘、只转动相机采集data/calibration/sentry_handeye；计算并检查外参后再更新现有配置。当前R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody仍是旧安装值，不能视为哨兵实测。

用户之前提供的机器人中心坐标为X前/Y左/Z上，光心(15,0,50) cm、机身(10,0,50) cm、炮口(10,0,45) cm；这些记录尚未应用，不能替代相机—云台手眼结果。
运行相机工具前沿用现场MVS环境变量（/opt/MVS/lib/64等），不复制SDK。
