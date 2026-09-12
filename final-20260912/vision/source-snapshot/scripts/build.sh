#!/bin/bash
# build.sh
# 清理旧的 build 目录并重新构建项目

set -e  # 出错时立即退出

# 删除旧的 build 和 cmake 缓存目录
rm -rf build cmake

# 重新生成构建目录
cmake -B build

# 使用所有 CPU 核心进行编译
make -C build/ -j"$(nproc)"

