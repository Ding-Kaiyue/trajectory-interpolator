#!/bin/bash

echo "=== 轨迹插值库安装脚本 ==="

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用sudo运行此脚本"
    exit 1
fi

# 检查依赖
echo "检查系统依赖..."
if ! command -v cmake &> /dev/null; then
    echo "安装cmake..."
    apt-get update
    apt-get install -y cmake build-essential
fi

# 编译安装
echo "编译安装轨迹插值库..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ROS2_MESSAGES=ON
make -j$(nproc)
make install

# 更新动态库缓存
echo "更新动态库缓存..."
ldconfig

echo "=== 安装完成 ==="
echo "轨迹插值库已成功安装到系统中"
echo ""
echo "使用示例:"
echo "  #include <trajectory_interpolator/trajectory_interpolator.hpp>"
echo "  # 编译时链接: -ltrajectory_interpolator_core" 