# Xiamen University RCS Team - Tracked Wheel-Leg Hybrid Infantry
# 厦门大学 RCS 战队 - 履带轮腿复合步兵机器人

![Team](https://img.shields.io/badge/Team-XMU%20RCS-blue)
![Robot](https://img.shields.io/badge/Robot-Tracked%20Wheel--Leg-orange)
![Chip](https://img.shields.io/badge/Chip-STM32F405-brightgreen)
![Framework](https://img.shields.io/badge/Framework-FreeRTOS-blueviolet)

## 📖 项目简介 (Introduction)
本项目为厦门大学 RCS 战队 **2026 赛季 RoboMaster 履带-轮腿复合式步兵机器人** 的底层嵌入式控制代码。

针对 RoboMaster 赛场上的复杂地形与高低地错落障碍（如楼梯、高地），本机器人创新采用了 **达妙电机驱动折叠轮腿 + 履带越障复合结构**。主控基于 **STM32F405**，搭载 **FreeRTOS** 实时操作系统，提供稳定、高频的整车运动学解算与状态机控制，不仅实现了基础底盘与云台控制，还深度集成了履带辅助越障、键鼠双轨控制、智能防卡弹火控以及高速 UART 串口外设通信架构。

---

## ✨ 核心特性 (Key Features)

- 🦿 **轮腿与履带协同控制**：通过键鼠/遥控器精确控制双膝关节高度（物理限幅保护），配合副履带电机涉水/爬楼梯，实现丝滑越障。
- 🛸 **混合底盘运动学逆解**：针对非对称的全向/麦轮混合底盘，重构了底层运动学模型，保证全向平移与旋转互不干涉。
- 🚫 **智能火控与防卡弹系统**：内置硬核防卡弹状态机。当拨盘电机过载超 1.5s，自动切入 2.1s 的反转退弹模式并无缝恢复，告别电机堵转烧毁。
- ⚡ **裁判系统动态功率调度**：实时解包裁判系统数据，在驱动电流下发前拦截超额功率，利用软件限幅保护电容和电池，杜绝超功率掉电。
- 📡 **UART 专线高速链路**：舍弃高延迟的 CAN 队列，转向 UART 高速直连（460800波特率视觉自瞄、115200波特率陀螺仪），极大释放 CAN 总线带宽用于电机高频收发。

---

## 🎮 键鼠与遥控器操作指南 (Controls)

本项目已实现完备的键鼠（PC）与遥控器（RC）双轨控制逻辑：

### ⌨️ 键盘与鼠标 (PC)
| 按键/操作 | 功能说明 |
| :--- | :--- |
| **W / A / S / D** | 控制底盘前后左右平移（带有软件加速度斜坡缓冲 `Ramp`）。 |
| **鼠标移动 (X/Y)**| 控制云台 Yaw（航向角）与 Pitch（俯仰角）跟随。 |
| **鼠标左键 / 右键**| 开火激活 / 摩擦轮开关及自瞄请求。 |
| **Z / C** | **[核心]** 控制折叠腿的高度升/降，联动调节左/右达妙膝关节电机位置。 |
| **X** | 切换底盘旋转（小陀螺模式）与跟随模式。 |
| **G** | 切换射击状态（调节裁判系统限流阈值的偏置，用于适应不同射击功率）。 |

### 🕹️ 遥控器 (RC)
* **左侧拨杆 / 右侧拨杆**：底盘平移控制与云台控制。
* **拨轮 / 开关 (S0/S1)**：切换 `NORMAL`（常规跟随）、`ROTATION`（小陀螺）、`AUTOAIM`（自瞄接管）模式。
* **自定义按键**：一键切换轮腿操作状态、开启履带电机爬楼 (`go_up`) 等。

---

## 🛠️ 开发环境与依赖 (Prerequisites)

准备以下工具链以进行二次开发：
1. **IDE**: 推荐使用 **Visual Studio 2022** + **VisualGDB** 插件（或 CLion / VSCode + CMake）。
2. **Toolchain**: **GNU Arm Embedded Toolchain** (如 `arm-none-eabi-gcc`)。
3. **烧录调试**: J-Link 或 DAP-Link 仿真器，以及配套的 OpenOCD / J-Link GDB Server。
4. **硬件**: 自研 STM32F405 主控板（需确认 UART 与 CAN 引脚映射）。

---

## 🚀 快速上手 (Quick Start)

### 1. 克隆代码库
git clone https://github.com/YourOrganization/Tracked-Wheel-Leg-Hybrid-Infantry.git cd Tracked-Wheel-Leg-Hybrid-Infantry

### 2. 编译项目 (以 VisualGDB 为例)
- 双击打开解决方案文件 (`.sln`)。
- 在工具栏选择正确的编译配置（如 `Debug` 或 `Release`）。
- 点击 `Build` (生成) -> `Build Solution` (生成解决方案) 进行编译。*(确保无 Error，Warning 可根据实际情况排查)*。

### 3. 烧录与调试
- 将 J-Link / DAP-Link 连接至主控板的 SWD 接口。
- 上电（注意 24V / 12V 供电安全，以及达妙电机和 DJI 电机的通信线连接）。
- 在 IDE 中点击 `Start Debugging (F5)`，程序将自动下载到单片机并停在 `main` 函数，点击 `Continue (F5)` 开始运行。

---

## 📂 主要目录结构 (Directory Structure)
📦 Tracked-Wheel-Leg-Hybrid-Infantry 
┣ 📂 basic/         # 硬件抽象层：CAN, Delay, UART, TIM 等底层初始化与回调 
	┣ 📂 device/            # 传感器与外设驱动 ┃ 
		┣ 📜 HTmotor.cpp      # 达妙伺服电机驱动 (MIT/POS/SPD 模式下发与解算) ┃ 
		┣ 📜 motor.cpp        # 大疆 M3508/M2006/M6020 无刷电机驱动与反馈 ┃ 
		┣ 📜 imu.cpp          # 陀螺仪 UART 通信解算 ┃ 
		┣ 📜 RC.cpp           # 遥控器与 DBUS / PC 键鼠链路数据解码 ┃ 
		┗ 📜 xuc_can.cpp      # 视觉上位机高速 UART 通信协议栈 
	┣ 📂 user/              # 核心业务与控制逻辑 ┃ 
		┣ 📜 taskslist.cpp    # FreeRTOS 任务调度大本营 (各类 Task 线程实体) ┃ 
		┣ 📜 control.cpp      # 状态机分发、全向底盘/云台运动学反解、限位保护 ┃ 
		┣ 📜 judgement.cpp    # 裁判系统串口数据结构体与解包 ┃ 
		┗ 📜 Power_limit.cpp  # 动态功率限幅及调度算法 
	┣ 📜 STM32F405.cpp      # main() 函数入口及全局对象实例化 
	┗ 📜 README.md          # 本文档

---

## ⏱️ 任务调度机制 (Task Scheduling)

| **任务名称**        | **周期** | **功能描述**                                                 |
| :------------------ | :------- | :----------------------------------------------------------- |
| **CanTransimtTask** | 2ms      | 单次循环集中发送大疆电机与达妙电机控制指流，保障总线利用率不拥堵。 |
| **MotorUpdateTask** | 2ms      | 执行 CAN 反馈解析，更新各底盘与云台电机数据，直接参与底盘功率限幅计算。 |
| **ControlTask**     | 2ms      | 运动学心跳核心。集成了键鼠操作转化、防卡弹反转状态机、云台底盘 PID 串级控制。 |
| **DecodeTask**      | 1ms      | 极高频数据解码。解包遥控器、DBUS、超级电容、裁判系统、IMU与视觉环形队列。 |
| **UiSendTask**      | 30ms     | 专用交互界面任务。定时下发自定义 UI 数据给裁判系统客户端选手端。 |
| **ArmTask**         | 20~300ms | 负责安全启动交互。检测遥控器/键鼠使能指令，分步激活达妙精密电机并自动检错重试。 |

---

## 📝 常见修改指南 (How to Modify)

* **修改 PID 参数**：在 `STM32F405.cpp` 的 `can1_motor` 和 `can2_motor` 全局数组中直接修改初始化参数。
* **新增 FreeRTOS 任务**：在 `taskslist.cpp` 中编写任务函数，并在 `void start_task()` 中调用 `xTaskCreate` 注册，确保合理分配栈大小 (`_STK_SIZE`) 与优先级。
* **改动控制键位**：定位至 `RC.cpp` 中的 `RC::OnPC()` 或 `RC::OnRC()` 函数，对 `pc.X/Y/Z/CTRL/SHIFT` 标志位逻辑进行调整。
* **调整达妙电机物理限位**：请严格检查 `RC::OnPC()` 与 `CONTROL::CHASSIS::Update()` 中的 `DMmotor[0].setPos` 和 `DMmotor[1].setPos` 防止机械碰撞。

---

## 🤝 参与贡献 (Contributing)

1. 新功能开发请新建分支 (`git checkout -b feature/your-feature-name`)。
2. 提交前请确保代码通过编译，并尽量进行实车空转/下地测试。
3. 遵循现有的代码风格，**重要逻辑修改必须添加中文注释**。
4. Push 后可向 `main` 分支发起 Pull Request。

本代码供技术交流及厦大 RCS 战队本队传承使用。

**Code with Passion by XMU RCS Team!**欢迎各高校战队共同探讨创新的复合轮腿架构算法！