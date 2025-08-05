#!/bin/bash

echo "=== trajectory_interpolator 编译脚本 ==="

# 检查是否在正确的目录
if [ ! -f "CMakeLists.txt" ]; then
    echo "错误: 请在项目根目录运行此脚本"
    exit 1
fi

# 解析命令行参数
BUILD_TYPE=${1:-Release}
ENABLE_ROS2=${2:-ON}

echo "构建类型: $BUILD_TYPE"
echo "ROS2支持: $ENABLE_ROS2"

# 创建构建目录
echo "创建构建目录..."
mkdir -p build
cd build

# 清理之前的构建
echo "清理之前的构建..."
rm -rf *

# 配置项目
echo "配置项目..."
if [ "$ENABLE_ROS2" = "ON" ]; then
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DUSE_ROS2_MESSAGES=ON
else
    cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DUSE_ROS2_MESSAGES=OFF
fi

if [ $? -ne 0 ]; then
    echo "错误: CMake配置失败"
    exit 1
fi

# 编译项目
echo "编译项目..."
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "错误: 编译失败"
    exit 1
fi

echo "=== 编译成功！ ==="
echo "库文件位置: build/lib/libtrajectory_interpolator_core.so"
echo "示例程序: build/bin/basic_interpolation_example"

# 显示文件信息
echo ""
echo "=== 生成的文件 ==="
ls -la bin/
ls -la lib/

echo ""
echo "=== 运行测试 ==="
if [ -d tests ]; then
    echo "==== 运行单元测试 ===="
    for test_file in tests/test_*; do
        if [ -f "$test_file" ] && [ -x "$test_file" ]; then
            echo "运行测试: $(basename "$test_file")"
            ./"$test_file"
            echo ""
        fi
    done
fi

echo ""
echo "=== 运行示例 ==="
if [ -f "bin/basic_interpolation_example" ]; then
    echo "运行示例程序..."
    ./bin/basic_interpolation_example
fi

echo ""
echo "=== 使用说明 ==="
echo "运行示例: ./build/bin/basic_interpolation_example"
echo "运行测试: ./build/tests/test_*"
echo ""
echo "在你的MoveIt规划节点中使用:"
echo "#include \"trajectory_interpolator/trajectory_interpolator.hpp\""
echo "std::unique_ptr<trajectory_interpolator::TrajectoryInterpolator> interpolator_;"