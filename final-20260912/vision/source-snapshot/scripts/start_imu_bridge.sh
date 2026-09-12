#!/bin/bash
# ============================================================================
#  IMU → CAN 转发器 一键启动脚本
#  ----------------------------------------------------------------------------
#  构型：IMU 直接连接上位机（无 C 板）。标定工具 capture 通过 CBoard(CAN)
#  读取四元数，因此需要先把 IMU 串口数据桥接到虚拟 CAN 接口 can0。
#
#  流程：
#    1) 若 can0 不存在，创建 vcan 虚拟接口（需 sudo，会提示密码）
#    2) 启动 ./imu_to_can 转发器（前台运行，Ctrl+C 停止）
#
#  用法：
#    ./start_imu_bridge.sh                 # 默认 /dev/gimbal 115200 can0 0x150
#    ./start_imu_bridge.sh /dev/ttyUSB1 115200 can0 0x150
# ============================================================================
set -e
DEV="${1:-/dev/gimbal}"
BAUD="${2:-115200}"
IFACE="${3:-can0}"
ID="${4:-0x150}"

# 防重复运行：已有 imu_to_can 在跑时拒绝再启动。
# 两个桥进程抢同一个串口会把 IMU 帧打碎，导致姿态数据乱跳（出现过）。
if pgrep -f 'imu_to_can' >/dev/null 2>&1; then
    echo "[start_imu_bridge] 已有 imu_to_can 进程在运行，拒绝再启动一个！" >&2
    echo "[start_imu_bridge] 运行中的进程 (PID + 命令)：" >&2
    pgrep -af 'imu_to_can' >&2
    pids=$(pgrep -f 'imu_to_can' | tr '\n' ' ')
    echo "[start_imu_bridge] 关闭上一个桥，请执行：kill $pids" >&2
    exit 1
fi

echo "[start_imu_bridge] 检查虚拟 CAN 接口 $IFACE ..."
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[start_imu_bridge] 接口不存在，创建 vcan $IFACE（需要 sudo 密码）"
    sudo ip link add "$IFACE" type vcan
    sudo ip link set "$IFACE" up
    echo "[start_imu_bridge] $IFACE 已创建并启用"
else
    echo "[start_imu_bridge] 接口 $IFACE 已存在，无需创建"
fi

echo "[start_imu_bridge] 启动 IMU→CAN 转发器: $DEV @ $BAUD -> $IFACE / 0x$ID"
echo "[start_imu_bridge] Ctrl+C 停止。日志实时打印到本终端。"
exec ./imu_to_can "$DEV" "$BAUD" "$IFACE" "$ID"
