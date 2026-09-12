# IMU 解析器（IMU Parser）

这是一个**独立的 IMU 串口数据解析程序**，用于解析通过 USB 串口（CH340）连接
到电脑的 IMU 姿态数据。

> ⚠️ 本程序完全独立于 `sp_vision_25` 项目源码，不依赖项目任何头文件 / 库，
> 也未改动 `sp_vision_25` 的任何源代码。它只是用来**读取并解析 IMU 数据**，
> 输出四元数 / 欧拉角 / 加速度，供调试、验证或后续接入其他流程使用。

## 背景

机器上的 IMU（维特智能系列，帧头 `5A A5`）通过 CH340 转串口直接连电脑。
系统识别为 `ttyUSB1`，`/dev/gimbal` 软链指向它。IMU 波特率 **115200**，帧率 **100Hz**。

`sp_vision_25` 项目里现有的 IMU 解析类（`DM_IMU` 达妙 0x55 协议、`CBoard` CAN 协议）
都不认识这个 `5A A5` 协议，因此本项目作为**源代码之外的独立解析工具**提供支持。

## 串口协议（实测）

| 参数 | 值 |
|---|---|
| 波特率 | 115200, 8N1 |
| 帧率 | 100 Hz |
| 帧长 | 82 字节，帧头 `0x5A 0xA5` |
| 数据 | float32 小端 |

帧布局（82 字节）：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0x00 | `5A A5` | 帧头 |
| 0x02 | `4C 00` | 固定 |
| 0x04 | `xx xx` | CRC / 校验（未验证） |
| 0x0E | uint32 | 时间戳（每帧 +10） |
| 0x12 | float32 ×3 | 加速度 AccX/Y/Z（g，静止 Z≈-1.0） |
| 0x22 | float32 ×3 | 角速度 GyroX/Y/Z |
| 0x36 | float32 | Yaw（度） |
| 0x3A | float32 | Roll（度） |
| 0x3E | float32 | Pitch（度） |
| 0x42 | float32 ×4 | **四元数 w、x、y、z** |
| 0x52 | — | 帧结束 |

## 编译

```bash
g++ -O2 -Wall -o imu_parser imu_parser.cpp
```

## 运行

```bash
./imu_parser                     # 默认读 /dev/gimbal @ 115200
./imu_parser /dev/ttyUSB1        # 指定串口设备
./imu_parser /dev/gimbal 115200  # 指定设备 + 波特率
./imu_parser /dev/gimbal 115200 --raw   # 紧凑输出，方便脚本/管道处理
./imu_parser --file dump.bin     # 离线回放串口抓包，验证解析逻辑
```

### 默认输出（每帧一行）

```
ts=982222  q=(w0.0050 x-0.0021 y0.9999 z-0.0109)  Yaw=179.42  Pitch=0.230  Roll=-1.245  Acc=(-0.0083 -0.0213 -0.9994)  |q|=1.0000
```

### `--raw` 输出（stdout 只有数据行，banner/统计走 stderr）

```
<时间戳> <w> <x> <y> <z> <yaw> <pitch> <roll> <accx> <accy> <accz>
```

四元数顺序为 **w, x, y, z**。

## 已验证

- 离线回放抓包：499 帧全部正确解析，四元数归一化 |q| = 1.0000
- 实时串口：100Hz 连续稳定输出，Yaw≈179.4°，AccZ≈-1.0g（静止）
