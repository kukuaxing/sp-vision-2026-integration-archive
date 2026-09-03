# STM32F405 XUC V2 独立构建状态

日期：2026-09-02

## 结论

在不安装 VisualGDB、也不修改电控原始压缩包的前提下，独立副本已能生成
STM32F405RGT6 的 ELF、BIN 和 HEX。构建过程只生成本地文件，未复位或写入
实车主控。

## 固定环境

- 编译器：GNU Arm Embedded Toolchain 10.3-2021.10，GCC 10.3.1。
- CMSIS、启动文件和 FreeRTOS：STM32CubeF4 v1.24.1，提交
  `b5abca20c9676b04f8d2885a668a9b653ee65705`。
- 旧 CAN HAL：STM32CubeF4 v1.14.0 兼容包，提交
  `b6e8197518e97d3fe9243fe49a692033b9d7d734`。
- 目标：STM32F405RG、Cortex-M4、Thumb、soft-float、1 MiB Flash、
  128 KiB 常规 SRAM。

选择旧 HAL 是必要的：电控代码使用 `CanTxMsgTypeDef`、`pTxMsg` 和
`HAL_CAN_Transmit`。STM32CubeF4 v1.24.1 已删除这些接口，不能在未做整车
CAN 回归的情况下直接升级。

## 2026-09-02 Release 构建

- ELF SHA256：`909786E7065A1B096FB3F5A03C45877F0B444210C944AA3D9C8F1C12AE8BAB8D`
- BIN SHA256：`2187F84D3F273FB4D8441A053F6D069C4858AAD9B500D515D9AD4C855755AAFF`
- HEX SHA256：`814DC7769FB2DC529993429FBD5CB2BF11D591276646E0F39BF63B0FE9A1D784`
- `.text + .rodata + .data` 约 39.7 KiB。
- `.bss + .data` 约 25.4 KiB；另预留 1.5 KiB heap/stack 检查区。
- 初始栈顶：`0x20020000`。
- Reset 向量：`0x08007199`，位于片内 Flash 且 Thumb 位有效。
- 已确认符号：`Reset_Handler`、`main`、`UART4_IRQHandler`、
  `USART2_IRQHandler`、`SysTick_Handler`、`vTaskStartScheduler` 和 XUC 解析器。

每次运行 `tools/build_firmware.ps1` 都会重新检查编译器版本、两个依赖提交、
文件哈希、向量表和关键符号，并生成 `build-arm/build-manifest.txt`。

## 尚未完成

- 尚未备份实车当前 1 MiB Flash。
- 尚未将新固件烧入主控。
- 已在实车验证 USART6 PC6/PC7、460800、100 Hz 上行帧和 IMU 弧度反馈。
- 基线工程存在原有编译警告；它们不是本次 XUC 修改引入的，但烧录前仍需
  结合实车回归判断。

在执行备份或烧录前，必须清空弹丸、断开发射机构动力、架空或固定底盘和
云台，并安排人员守在急停/总电源旁。
