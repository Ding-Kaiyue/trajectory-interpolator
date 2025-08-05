# 发布说明

本文档记录轨迹插值库的版本发布信息和重要更新。

## 版本 1.0.0 (2024-08-05)

### 🎉 首次发布

这是轨迹插值库的首次正式发布，提供了完整的机器人臂轨迹插值功能。

#### ✨ 新功能

- **高性能轨迹插值**: 基于 `ttk592/spline` 库的三次样条插值算法
- **MoveIt集成**: 支持MoveIt轨迹数据结构的直接集成
- **智能指针通信**: 通过 `std::unique_ptr` 实现高效的节点间通信
- **ROS2支持**: 可选支持ROS2消息类型，便于在ROS2环境中使用
- **约束检查**: 内置速度、加速度、加加速度约束检查
- **实时插值**: 支持任意时间点的位置、速度、加速度查询

#### 🏗️ 核心组件

- **TrajectoryInterpolator**: 高级插值器接口，提供用户友好的API
- **MoveItSplineAdapter**: 底层适配器，直接与样条库交互
- **SplineConfig**: 配置结构，支持多种插值类型和约束参数

#### 📊 数据结构

- **Trajectory**: 内部轨迹数据结构，支持多关节轨迹
- **TrajectoryPoint**: 轨迹点数据结构，包含位置、速度、加速度信息
- **SplineConfig**: 插值配置和约束参数，支持灵活配置

#### 🛠️ 构建系统

- **CMake配置**: 完整的CMake构建系统
- **条件编译**: 支持ROS2消息的条件编译
- **单元测试**: 完整的Google Test测试套件
- **示例程序**: 基本使用示例

#### 📚 文档

- **API文档**: 完整的API参考文档
- **使用教程**: 从基础到高级的使用指南
- **开发指南**: 贡献代码和开发环境设置
- **安装说明**: 详细的安装和配置指南

#### 🧪 测试覆盖

- **核心功能测试**: 16个测试用例，覆盖所有核心功能
- **MoveIt兼容性测试**: 7个测试用例，验证MoveIt集成
- **错误处理测试**: 验证异常情况的正确处理
- **性能测试**: 验证插值性能和内存使用

#### 🔧 技术特性

- **C++17标准**: 使用现代C++特性
- **智能指针**: 自动内存管理
- **异常安全**: 强异常安全保证
- **线程安全**: 支持多线程环境
- **跨平台**: 支持Linux、Windows、macOS

#### 📦 发布包

- **源码包**: `trajectory_interpolator_v1.0.0.tar.gz`
- **Debian包**: 
  - `libtrajectory-interpolator0_1.0.0-1_amd64.deb`
  - `libtrajectory-interpolator-dev_1.0.0-1_amd64.deb`

#### 🚀 快速开始

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"

int main() {
    auto interpolator = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
    
    // 配置插值参数
    trajectory_interpolator::SplineConfig config;
    config.dt = 0.02;
    config.spline_type = SplineConfig::CSPLINE;
    interpolator->setInterpolationConfig(config);
    
    // 加载轨迹并插值
    if (interpolator->loadTrajectory(trajectory)) {
        auto positions = interpolator->interpolateAtTime(1.5);
        auto velocities = interpolator->getVelocityAtTime(1.5);
        auto accelerations = interpolator->getAccelerationAtTime(1.5);
    }
    
    return 0;
}
```

#### 📋 系统要求

- **C++17** 编译器 (GCC 7+, Clang 5+, MSVC 2017+)
- **CMake 3.8+**
- **GTest** (用于单元测试)
- **ROS2 Humble** (可选，用于ROS2消息支持)

#### 🔗 依赖项

- **[ttk592/spline](https://github.com/ttk592/spline)**: 样条插值算法 (GPL-2.0)
- **Google Test**: 单元测试框架
- **ROS2**: 机器人操作系统 (可选)

#### 📄 许可证

- **主项目**: MIT License
- **第三方组件**: GPL-2.0 (spline.h)

#### 🙏 致谢

- [ttk592/spline](https://github.com/ttk592/spline) - 提供高性能样条插值算法
- MoveIt社区 - 提供机器人轨迹规划框架
- ROS2社区 - 提供机器人操作系统

---

## 版本历史

### 开发版本

#### v0.1.0-alpha (2024-08-01)
- 初始原型实现
- 基本样条插值功能
- 简单的API设计

#### v0.2.0-beta (2024-08-03)
- 添加MoveIt适配器
- 实现约束检查功能
- 完善错误处理

#### v0.3.0-rc (2024-08-04)
- 添加ROS2消息支持
- 完善单元测试
- 优化性能

#### v1.0.0 (2024-08-05)
- 正式发布版本
- 完整的文档体系
- 生产就绪的代码质量

---

## 升级指南

### 从开发版本升级

如果你正在使用开发版本，建议升级到正式版本：

1. **备份当前代码**
   ```bash
   cp -r your_project your_project_backup
   ```

2. **更新依赖**
   ```bash
   # 更新CMakeLists.txt中的版本号
   # 更新头文件包含路径（如果有变化）
   ```

3. **重新编译**
   ```bash
   rm -rf build
   ./build.sh
   ```

4. **运行测试**
   ```bash
   cd build
   ./tests/test_trajectory_interpolator
   ./tests/test_moveit_compatibility
   ```

### API变更

v1.0.0 是第一个正式版本，没有向后兼容性问题。

---

## 已知问题

### 编译警告

- **警告**: `warning: type violates the C++ One Definition Rule`
  - **原因**: 由 `spline.h` 中的匿名命名空间引起
  - **影响**: 不影响功能，仅为警告
  - **解决方案**: 已在CMakeLists.txt中添加 `-Wno-subobject-linkage` 抑制

### 性能考虑

- **内存使用**: 大型轨迹可能占用较多内存
  - **建议**: 及时调用 `clear()` 释放内存
- **计算开销**: 高精度插值可能较慢
  - **建议**: 根据需求调整时间步长

---

## 未来计划

### v1.1.0 (计划中)
- [ ] 添加更多插值算法
- [ ] 优化内存使用
- [ ] 添加Python绑定
- [ ] 改进错误处理

### v1.2.0 (计划中)
- [ ] 添加并行插值支持
- [ ] 实现GPU加速
- [ ] 添加可视化工具
- [ ] 完善性能分析

---

**最后更新**: 2024-08-05 