# RM_Infantry-Robot

RoboMaster（机甲大师）步兵机器人主控固件。基于 STM32F405，运行 FreeRTOS，使用 C++（HAL 库）开发。

## 硬件平台

| 项目 | 说明 |
|------|------|
| 主控 | STM32F405RGT6 |
| 实时系统 | FreeRTOS |
| 开发环境 | VisualGDB + Visual Studio（GCC 工具链） |
| 语言 | C++（STM32 HAL 库） |

## 硬件映射

| 外设 | 接口 | 波特率 | 用途 |
|------|------|--------|------|
| CAN1 | PA11/PA12 | 1 Mbps | 2× M3508 摩擦轮、1× M2006 拨弹轮、1× 达妙 J4310-2EC pitch 电机（ID=3，位置速度模式） |
| CAN2 | PB12/PB13 | 1 Mbps | 4× M3508 底盘电机、1× M6020 yaw 云台电机 |
| USART2 | — | 921600 | CH010 陀螺仪（云台 IMU） |
| USART3 | — | 100000 | DR16 遥控器 |
| USART5 | — | 9600 | 电源 / 裁判系统 |
| USART6 | PC6/PC7 | 460800 | 小电脑 XUC V2（CH340，14B 下行/20B 上行） |

## 功能模块

- **底盘**：45° 全向轮速度解算、底盘-云台分离坐标变换、小陀螺模式
- **云台**：yaw 轴 IMU 锁角、pitch 轴达妙 DM 电机位置控制（`[-0.25, 0.4]` rad 软限位）
- **发射**：摩擦轮转速闭环 + 拨弹供弹，遥控触发
- **小电脑通信**：USART6 周期发送 IMU 反馈并解析目标角；Phase 2A 只允许受门控的 ±1° yaw 测试
- **电机**：支持 M3508 / M6020 / M2006（DJI 协议）与达妙 DM 电机（MIT / 位置速度模式）

## 遥控操作

- 右摇杆左右（ch0）→ yaw；右摇杆上下（ch1）→ pitch
- 左摇杆 → 底盘前后 / 平移自转
- 鼠标左键 → 发射
- 两拨杆组合 → 切换工作模式（RESET / SEPARATE / ROTATION / FOLLOW / LOCK / ROTATION_FREE）

## 构建

使用 VisualGDB 打开 `FreeRTOS.sln` 编译并烧录到 STM32F405。

也可运行 `tools/build_firmware.ps1`，使用固定的 GNU Arm Embedded
Toolchain 10.3.1、STM32CubeF4 v1.24.1 的 CMSIS/FreeRTOS，以及兼容旧 CAN
API 的 STM32CubeF4 v1.14.0 HAL 生成 ELF、BIN 与 HEX。该独立构建流程只
生成文件，不连接或烧录主控。HAL 版本不可擅自升级，否则会把电控工程使用
的 `CanTxMsgTypeDef`/`HAL_CAN_Transmit` 接口替换为不兼容的新接口。

## 烧录前备份

`tools/backup_stm32f405_flash.ps1` 使用 CMSIS-DAP 和 OpenOCD 读取并校验
完整的 1 MiB 片内 Flash。脚本要求显式提供 `-ConfirmRobotIsSafe`，因为备份
过程中会短暂复位并暂停主控。该脚本不擦除或写入 Flash。

## XUC V2 首次接板限制

- 协议包头为 `SP`，小电脑到下位机 14 字节，下位机到小电脑 20 字节。
- CRC16 初值为 `0xFFFF`，CRC 低字节在前。
- 下位机以 100 Hz 主动发送反馈，IMU 角度由度转换为弧度。
- Phase 2A 只有在遥控器、严格 CRC IMU、XUC 新鲜度和显式拨杆边沿全部通过时，
  才允许 yaw 在解锁点 ±1° 内以不超过 5°/s 运动。
- pitch 与发射机构仍不接收 XUC 命令；详见 `docs/XUC_V2_PHASE2A_BOUNDED_YAW.md`。
- 实车已确认使用 USART6/460800、PC6/PC7。

## 目录结构

```
STM32F405/
├── can.cpp/.h        # CAN 收发
├── motor.cpp/.h      # DJI 电机（M3508/6020/2006）位置-速度-电流级联
├── HTmotor.cpp/.h    # 达妙 DM 电机（J4310 等）
├── control.cpp/.h    # 底盘 / 云台 / 发射控制
├── imu.cpp/.h        # IMU601 / CH010 / HI226 陀螺仪
├── RC.cpp/.h         # DR16 遥控器解码与模式映射
├── taskslist.cpp     # FreeRTOS 任务调度
└── ...
```
