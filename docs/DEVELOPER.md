# 开发指南

本指南介绍如何为轨迹插值库贡献代码和设置开发环境。

## 开发环境设置

### 系统要求
- Ubuntu 20.04+ 或类似Linux发行版
- C++17 兼容的编译器 (GCC 7+, Clang 5+)
- CMake 3.8+
- Git

### 依赖安装
```bash
# 基础开发工具
sudo apt update
sudo apt install build-essential cmake git

# 测试框架
sudo apt install libgtest-dev

# ROS2 (可选，用于ROS2消息支持)
sudo apt install ros-humble-ament-cmake ros-humble-rclcpp ros-humble-trajectory-msgs ros-humble-moveit-msgs ros-humble-builtin-interfaces
```

### 获取源码
```bash
git clone https://github.com/Ding-Kaiyue/trajectory-interpolator.git
cd trajectory-interpolator
```

## 构建项目

### 快速构建
```bash
./build.sh
```

### 手动构建
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS2_MESSAGES=ON
make -j$(nproc)
```

### 构建选项
- `CMAKE_BUILD_TYPE`: Debug, Release, RelWithDebInfo
- `USE_ROS2_MESSAGES`: ON/OFF - 启用ROS2消息支持
- `BUILD_TESTS`: ON/OFF - 构建测试

## 运行测试

```bash
# 运行所有测试
cd build
./tests/test_trajectory_interpolator
./tests/test_moveit_compatibility

# 使用CTest
ctest --verbose
```

## 代码规范

### C++编码规范
- 使用C++17标准
- 遵循Google C++ Style Guide
- 类名使用PascalCase
- 函数和变量使用camelCase
- 常量使用UPPER_SNAKE_CASE

### 文件组织
```
src/                    # 源文件
├── trajectory_interpolator.cpp
└── moveit_spline_adapter.cpp

include/trajectory_interpolator/  # 头文件
├── trajectory_interpolator.hpp
├── moveit_spline_adapter.hpp
├── config.hpp
└── spline.h

tests/                  # 测试文件
├── test_trajectory_interpolator.cpp
└── test_moveit_compatibility.cpp

examples/               # 示例文件
└── basic_interpolation_example.cpp
```

### 注释规范
```cpp
/**
 * @brief 轨迹插值器类
 * 
 * 提供高性能的机器人臂轨迹插值功能，支持MoveIt集成。
 * 
 * @example
 * ```cpp
 * auto interpolator = std::make_unique<TrajectoryInterpolator>();
 * interpolator->loadTrajectory(trajectory);
 * auto positions = interpolator->interpolateAtTime(1.5);
 * ```
 */
class TrajectoryInterpolator {
public:
    /**
     * @brief 在指定时间点插值
     * @param time 插值时间点
     * @return 关节位置向量
     * @throws std::runtime_error 当时间超出范围时
     */
    std::vector<double> interpolateAtTime(double time);
};
```

## 添加新功能

### 1. 创建功能分支
```bash
git checkout -b feature/new-feature-name
```

### 2. 实现功能
- 在相应源文件中添加实现
- 在头文件中添加声明
- 添加必要的测试

### 3. 编写测试
```cpp
TEST(TrajectoryInterpolatorTest, NewFeature) {
    auto interpolator = std::make_unique<TrajectoryInterpolator>();
    
    // 设置测试数据
    trajectory_interpolator::SplineConfig config;
    interpolator->setInterpolationConfig(config);
    
    // 执行测试
    // ...
    
    // 验证结果
    EXPECT_TRUE(result);
}
```

### 4. 更新文档
- 更新API文档
- 添加使用示例
- 更新变更日志

## 提交代码

### 提交前检查
```bash
# 运行测试
./build.sh
cd build && make test

# 检查代码格式
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format --dry-run

# 静态分析
cppcheck --enable=all src/ include/
```

### 提交规范
```bash
git add .
git commit -m "feat: add new interpolation method

- Add cubic Hermite spline interpolation
- Improve performance by 20%
- Add comprehensive unit tests
- Update documentation"
```

### 提交类型
- `feat`: 新功能
- `fix`: 修复bug
- `docs`: 文档更新
- `style`: 代码格式
- `refactor`: 重构
- `test`: 测试相关
- `chore`: 构建工具等

## 发布流程

### 1. 版本号更新
- 更新 `CMakeLists.txt` 中的版本号
- 更新 `debian/changelog`
- 更新 `docs/CHANGELOG.md`

### 2. 创建发布包
```bash
# 创建源码包
./create_release_package.sh 1.1.0

# 创建Debian包
./build_debian.sh
```

### 3. 发布到GitHub
- 创建GitHub Release
- 上传发布包
- 更新文档

## 问题报告

### Bug报告模板
```markdown
**描述**
简要描述bug

**重现步骤**
1. 步骤1
2. 步骤2
3. 步骤3

**期望行为**
描述期望的行为

**实际行为**
描述实际的行为

**环境信息**
- 操作系统: Ubuntu 20.04
- 编译器: GCC 9.4.0
- 版本: 1.0.0

**附加信息**
任何其他相关信息
```

## 联系方式

- **Email**: kaiyue.ding@raysense.com
- **GitHub Issues**: [提交问题](https://github.com/Ding-Kaiyue/trajectory-interpolator/issues)
- **微信**: d18292819833

---

**注意**: 贡献代码前请确保已阅读并同意项目的MIT许可证。 