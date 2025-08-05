# 轨迹插值库

一个基于 `ttk592/spline` 库的高性能C++轨迹插值库，专为机器人臂轨迹规划设计，支持MoveIt集成。

## 快速安装

### 方法1: 源码安装（推荐）

```bash
# 克隆仓库
git clone https://github.com/Ding-Kaiyue/trajectory-interpolator.git
cd trajectory-interpolator

# 编译安装
./build.sh
```

### 方法2: 发布包安装

```bash
# 下载发布包
wget https://github.com/Ding-Kaiyue/trajectory-interpolator/releases/download/v1.0.0/trajectory_interpolator_v1.0.0.tar.gz

# 解压并安装
tar -xzf trajectory_interpolator_v1.0.0.tar.gz
cd trajectory_interpolator_release
sudo ./install.sh
```

### 方法3: Debian包安装

```bash
# 下载并安装
wget https://github.com/Ding-Kaiyue/trajectory-interpolator/releases/download/v1.0.0/libtrajectory-interpolator0_1.0.0_amd64.deb
wget https://github.com/Ding-Kaiyue/trajectory-interpolator/releases/download/v1.0.0/libtrajectory-interpolator-dev_1.0.0_amd64.deb
sudo dpkg -i libtrajectory-interpolator0_1.0.0_amd64.deb
sudo dpkg -i libtrajectory-interpolator-dev_1.0.0_amd64.deb
sudo apt-get install -f
```

## 快速开始

```cpp
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include <iostream>

int main() {
    // 创建插值器
    auto interpolator = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
    
    // 配置插值参数
    trajectory_interpolator::SplineConfig config;
    config.dt = 0.02;
    config.spline_type = SplineConfig::CSPLINE;
    interpolator->setInterpolationConfig(config);
    
    // 创建轨迹数据
    trajectory_interpolator::Trajectory trajectory;
    trajectory.joint_names = {"joint1", "joint2", "joint3"};
    
    // 添加轨迹点
    trajectory_interpolator::TrajectoryPoint point1;
    point1.time_from_start = 0.0;
    point1.positions = {0.0, 0.0, 0.0};
    point1.velocities = {0.0, 0.0, 0.0};
    point1.accelerations = {0.0, 0.0, 0.0};
    
    trajectory_interpolator::TrajectoryPoint point2;
    point2.time_from_start = 3.0;
    point2.positions = {1.0, 1.0, 1.0};
    point2.velocities = {0.0, 0.0, 0.0};
    point2.accelerations = {0.0, 0.0, 0.0};
    
    trajectory.points = {point1, point2};
    
    // 加载轨迹
    if (interpolator->loadTrajectory(trajectory)) {
        std::cout << "轨迹加载成功！" << std::endl;
        
        // 在指定时间点插值
        double time = 1.5;
        auto positions = interpolator->interpolateAtTime(time);
        auto velocities = interpolator->getVelocityAtTime(time);
        auto accelerations = interpolator->getAccelerationAtTime(time);
        
        std::cout << "时间 " << time << "s 的插值结果:" << std::endl;
        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "关节 " << i << ": 位置=" << positions[i] 
                      << ", 速度=" << velocities[i] 
                      << ", 加速度=" << accelerations[i] << std::endl;
        }
        
        // 插值完整轨迹
        auto interpolated = interpolator->interpolate(0.01);
        std::cout << "插值轨迹包含 " << interpolated.points.size() << " 个点" << std::endl;
        
    } else {
        std::cerr << "轨迹加载失败！" << std::endl;
        return 1;
    }
    
    return 0;
}
```

## 编译

```bash
# 使用g++编译您的程序
g++ -std=c++17 -ltrajectory_interpolator_core -lpthread your_program.cpp -o your_program

# 使用pkg-config（如果已安装）
g++ -std=c++17 $(pkg-config --cflags --libs trajectory_interpolator) your_program.cpp -o your_program
```

## 主要功能

* **高性能插值**: 基于三次样条插值算法，提供C²连续的平滑轨迹
* **MoveIt兼容**: 支持MoveIt轨迹数据结构的直接集成
* **智能指针通信**: 通过 `std::unique_ptr` 实现高效的节点间通信
* **ROS2集成**: 可选支持ROS2消息类型，便于在ROS2环境中使用
* **约束检查**: 内置速度、加速度、加加速度约束检查
* **实时插值**: 支持任意时间点的位置、速度、加速度查询
* **单元测试**: 完整的测试覆盖，确保代码质量

## API概览

```cpp
// 创建和配置
auto interpolator = std::make_unique<trajectory_interpolator::TrajectoryInterpolator>();
interpolator->setInterpolationConfig(config);

// 加载轨迹
interpolator->loadTrajectory(trajectory);

// 实时插值
auto positions = interpolator->interpolateAtTime(time);
auto velocities = interpolator->getVelocityAtTime(time);
auto accelerations = interpolator->getAccelerationAtTime(time);

// 完整轨迹插值
auto interpolated = interpolator->interpolate(0.01);

// 约束检查
bool valid = interpolator->checkConstraints();

// 状态查询
bool loaded = interpolator->isLoaded();
double start_time = interpolator->getStartTime();
double end_time = interpolator->getEndTime();
```

## 依赖项

* **C++17** 编译器
* **CMake 3.8+**
* **GTest** (用于单元测试)
* **ROS2 Humble** (可选，用于ROS2消息支持)

## 故障排除

1. **编译错误**: 确保C++17支持，检查依赖项
2. **ROS2头文件未找到**: 确保ROS2环境已设置
3. **库文件未找到**: 运行 `sudo ldconfig`
4. **测试失败**: 检查GTest安装

## 获取帮助

如果您在使用过程中遇到问题，可以通过以下方式获取帮助：

### GitHub Issues

* **使用问题**: [提交使用问题](https://github.com/Ding-Kaiyue/trajectory-interpolator/issues/new?template=usage-question.md) - 提交使用中的问题，我们会提供指导
* **Bug 报告**: [报告 Bug](https://github.com/Ding-Kaiyue/trajectory-interpolator/issues/new?template=bug-report.md) - 报告发现的 Bug，我们会尽快修复
* **功能建议**: [提出建议](https://github.com/Ding-Kaiyue/trajectory-interpolator/issues/new?template=feature-request.md) - 提出新功能建议，我们会认真考虑

### 联系方式

* **Email**: kaiyue.ding@raysense.com
* **微信**: d18292819833
* **商业合作**: 欢迎联系进行定制开发和技术支持

## 📚 文档

更多文档请查看 [docs/](docs/) 目录，包括API参考、开发者指南等。

## 许可证

MIT License - 详见 LICENSE 文件

**注意**: 本项目包含来自 [ttk592/spline](https://github.com/ttk592/spline) 的 GPL-2.0 组件，仅用于样条插值算法实现。

## 致谢

* [ttk592/spline](https://github.com/ttk592/spline) - 提供高性能样条插值算法
* MoveIt社区 - 提供机器人轨迹规划框架
* ROS2社区 - 提供机器人操作系统

---

**注意**: 本库专为机器人轨迹插值设计，已在多种机器人平台上测试验证。