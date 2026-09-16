# 当前固件来源及配置

基础 SentryUC master 6a107ca08ab944c43043a4d861032b03101c71fc，来源 https://github.com/zhangjichuan1009-create/SentryUC 。
保留缺失返回修复、ST Legacy CAN/APB 构建适配、CAN 接收边界检查及反馈时间戳。
当前保存目标 pitch_zero.h 的 PITCH_ZERO_RAW=32685，约 -0.0316391 rad；是用户指定上抬 8° 后的角度，非重新测量的水平。
SENTRY_PITCH_ZERO_MODE=1 现表示专用云台标定控制器，SENTRY_CALIBRATION_READ_ONLY=0。
pitch_zero.cpp 负责 P 轴上电回归、到位后遥控接管、DM Yaw 低速遥控以及限幅/掉线/故障保护。原整车 ControlTask 在该模式跳过，避免原控制路径阻塞及相互覆盖。发送任务只允许 CAN1 0x109、CAN2 0x206 及只读温度诊断；底盘、发射、硬件置零继续屏蔽。
遥控左拨杆上/右拨杆下，先回中；P 为通道 3、Yaw 为通道 0。完整操作与当前实机验证范围见 FLASH_REPORT.md。
回归测试 tools/test_gimbal_controller.cpp 直接包含实际控制器，使用 tools/test_stubs 替代硬件依赖；本机以 g++ -std=c++17 -static -Itools/test_stubs tools/test_gimbal_controller.cpp -o build/test_gimbal_controller.exe 构建，分别无参数及带 outside 参数运行。
编译产物仅在 build/，未提交或推送。

现场串口修正：遥控接 USART2，100000/8E1（HAL 使用 9-bit word + even parity）。USART3 分配给裁判的 115200/8N1 通道，尚未验证裁判链路。遥控接收及手动模式进入已确认，Yaw CAN2 ID 0x06 无反馈，处于通信超时锁定。

CAN 接收分界修正为 14，过滤器在接收中断前初始化；错误回调恢复接收。Yaw 故障停止重试改为 500 ms，P 轴反馈约 162 Hz。增加仅接收的 CAN ID 盘点及默认不执行的只读寄存器 2 查询邮箱（地址 1～16）。查询不发使能或参数写入。最新证据见 FLASH_REPORT.md。

小 Y 手动辨识已确认对应 CAN1 0x205，现已加入编码器闭环，左摇杆左右控制。仅 CAN1 0x1FF 首个槽位放行，其余槽位为零；最大指令 2500，目标速度 0.10 rad/s、范围 ±0.18 rad。修复反馈时间戳的无符号减法误超时。最新版已烧录并完整读回，但当前所有 CAN 电机均无反馈、程序保持保护，尚未完成小 Y 运动验收；详见 FLASH_REPORT.md。


HI91解析修复仅完成源码和构建，尚未烧录：imu_packet.h按HiPNUC原始传感器坐标解析roll/pitch/yaw和wxyz四元数；不能把原始轴直接当作安装后的云台轴。tools/test_imu_packet.cpp为实际帧、CRC、截断及异常数值回归，可用g++ -std=c++14 -fsanitize=address,undefined构建运行。新IMU队列将实际长度与内容一起复制，未收到新帧时不解码，不影响遥控等原有字节队列。


现场接线已由用户确认：小Y IMU在UART3，大Y IMU在UART5。原固件错读USART1并让裁判系统占用USART3。接收侧波特率检查确认UART3在921600收到82字节HI91及CRC有效帧（115200/230400/460800未得到有效帧）；测试后已恢复寄存器。
源码已将专用标定模式的小Y IMU改为USART3 921600，大Y保留UART5 115200，禁用标定模式下裁判系统对USART3的占用。校验通过的小Y原始HI91帧经UART4 460800单向转发给小电脑，保留四元数与设备时间戳；DMA忙时跳过，不改写正在发送的缓冲区。尚待烧录后的回传和双轴响应验证；旧capture的XUC解析不兼容该HI91数据。
