#!/bin/bash
# start_spvision.sh
# 启动 sp_vision 程序，使用 SDK 自带 Qt 库环境

# 加载用户 bash 环境（包含 PATH、conda、环境变量等）
source /home/rm/.bashrc

# 设置 MVS SDK Qt 库环境
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

# 切换到程序所在目录（注意：用绝对路径）
cd /home/rm/test/sp_vision_25-main || exit 1

# 启动程序
/home/rm/test/sp_vision_25-main/build/standard configs/sentry.yaml >> /home/rm/spvision.log 2>&1 &

