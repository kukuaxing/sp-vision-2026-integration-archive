# XUC V2 第一阶段说明

本分支以电控提供的 `RM_Infantry-Robot-main.zip` 为基线，只增加小电脑通信初始化。

## 当前实现

- USART6，PC6(TX)/PC7(RX)，460800，8N1；
- 下位机接收 14 字节 `RxPacket_TJ`；
- 下位机以 100 Hz 发送 20 字节 `TxPacket_TJ`；
- 包头固定为 `SP`；
- CRC16 初值 `0xFFFF`，低字节在前；
- 接收器支持拆包、粘包、包头前噪声、CRC错误后的重同步；
- 非法 control/shoot、NaN、Inf 不会更新最新命令；
- 200 ms 内没有合法包时，`HasFreshCommand()` 返回 false；
- CH010 的角度按现有控制代码所用的度数转换成弧度后发送。

## 强制安全边界

第一阶段没有任何代码读取 `LatestCommand()` 后写入云台或发射机构：

- `control_TJ` 只被保存；
- `shoot_TJ` 只被保存；
- yaw/pitch 只被保存；
- 现有遥控器、IMU、CAN、电机和发射状态机保持原样。

## 当前反馈字段

- mode：0；
- robot_id：0；
- bullet_speed：0；
- bullet_count：0；
- imu_pitch/yaw：有效 CH010 角度转换为弧度；IMU 尚未成功解析时为 0。

这些安全默认值足以验证双向协议。模式、机器人 ID、弹速和弹丸计数需要在后续独立提交中接入可靠数据源。

## 主机侧测试

在源码根目录运行：

```bash
python3 tests/test_xuc_v2_protocol.py
```

如果有桌面 `g++`，还可以直接编译实际 `xuc.cpp` 中的 CRC 实现：

```bash
g++ -std=c++11 -Wall -Wextra -Werror \
  -I tests/host_stubs \
  -I STM32F405 \
  -include tests/host_stubs/xuc_host_stub.h \
  STM32F405/xuc.cpp \
  tests/xuc_v2_crc_test.cpp \
  -o xuc_v2_crc_test

./xuc_v2_crc_test
```

## 烧录前必须确认

1. 实车 CH340 确实连接 USART6 PC6/PC7，而不是 CAN 或其他 UART；
2. 串口参数确实为 460800、8N1；
3. CH340 与主控板共地，TX/RX 交叉连接；
4. 下位机已有供电时不使用 CH340 VCC 重复供电；
5. 用 VisualGDB/ARM GCC 完整编译通过。

第一次测试必须卸弹、断开发射机构动力并架空云台。先运行小电脑端只接收探测；确认持续收到合法 20 字节反馈后，再运行全零安全心跳。不要直接运行完整自瞄程序。
