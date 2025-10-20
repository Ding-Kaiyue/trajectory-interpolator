cmake_minimum_required(VERSION 3.8)
project(trajectory_interpolator)

set(CMAKE_CXX_STANDARD 17)

# Ensure consistent behavior for option() vs normal variables (suppress CMP0077 developer warnings)
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-subobject-linkage)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 默认禁用测试
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF)
endif()

# 查找GTest，如果需要启用测试则需要GTest
find_package(GTest QUIET)
if(BUILD_TESTING AND NOT GTest_FOUND)
  message(WARNING "BUILD_TESTING enabled but GTest not found, tests will be disabled")
  set(BUILD_TESTING OFF)
endif()

set(BUILD_TESTS ${BUILD_TESTING})

find_package(Threads REQUIRED)

# === trajectory_interpolator 核心库（无ROS2依赖） ===
add_library(trajectory_interpolator_core SHARED
  src/moveit_spline_adapter.cpp
  src/trajectory_interpolator.cpp
)

target_link_libraries(trajectory_interpolator_core 
  Threads::Threads
)

target_include_directories(trajectory_interpolator_core 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 设置库的版本信息
set_target_properties(trajectory_interpolator_core PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
  OUTPUT_NAME "trajectory_interpolator_core"
)

# 设置库的输出目录到lib文件夹
set_target_properties(trajectory_interpolator_core PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# === ROS2节点（可选，如果需要ROS2功能） ===
# 只有在需要ROS2功能时才启用
option(USE_ROS2_MESSAGES "Enable ROS2 message support" ON)

if(USE_ROS2_MESSAGES)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  
  # 为ROS2版本添加编译定义和依赖
  target_compile_definitions(trajectory_interpolator_core PRIVATE USE_ROS2_MESSAGES)
  ament_target_dependencies(trajectory_interpolator_core 
    trajectory_msgs 
    moveit_msgs 
    builtin_interfaces
  )
else()
  target_compile_definitions(trajectory_interpolator_core PRIVATE USE_ROS2_MESSAGES=0)
endif()

# === 示例程序 ===
add_executable(basic_interpolation_example examples/basic_interpolation_example.cpp)
target_link_libraries(basic_interpolation_example
  trajectory_interpolator_core
  Threads::Threads
)
target_include_directories(basic_interpolation_example 
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 为示例程序添加相同的编译定义
if(USE_ROS2_MESSAGES)
  target_compile_definitions(basic_interpolation_example PRIVATE USE_ROS2_MESSAGES)
else()
  target_compile_definitions(basic_interpolation_example PRIVATE USE_ROS2_MESSAGES=0)
endif()

set_target_properties(basic_interpolation_example PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# === 安装头文件和库 ===
install(DIRECTORY include/trajectory_interpolator DESTINATION include)

# 创建库的别名
add_library(trajectory_interpolator ALIAS trajectory_interpolator_core)

# 设置安装时的库名称
set_target_properties(trajectory_interpolator_core PROPERTIES
  LIBRARY_OUTPUT_NAME "trajectory_interpolator_core"
  ARCHIVE_OUTPUT_NAME "trajectory_interpolator_core"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 安装目标
install(TARGETS
  trajectory_interpolator_core
  basic_interpolation_example
  EXPORT trajectory_interpolatorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# === 测试配置 ===
if(BUILD_TESTS)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

  file(GLOB TEST_SOURCES tests/*.cpp)
  if(TEST_SOURCES)
    foreach(TEST_FILE ${TEST_SOURCES})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
      
      add_executable(${TEST_NAME} ${TEST_FILE})
      
      set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
      )
      
      # 只有在GTest可用时才链接GTest库
      if(GTest_FOUND)
        target_link_libraries(${TEST_NAME}
          trajectory_interpolator_core
          GTest::gtest
          GTest::gmock
          GTest::gtest_main
          GTest::gmock_main
          Threads::Threads
        )
      else()
        target_link_libraries(${TEST_NAME}
          trajectory_interpolator_core
          Threads::Threads
        )
      endif()
      
      # 为测试程序添加相同的编译定义
      if(USE_ROS2_MESSAGES)
        target_compile_definitions(${TEST_NAME} PRIVATE USE_ROS2_MESSAGES)
      else()
        target_compile_definitions(${TEST_NAME} PRIVATE USE_ROS2_MESSAGES=0)
      endif()
      
      target_include_directories(${TEST_NAME} 
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
      )
      
      # 只有在GTest可用时才添加测试
      if(GTest_FOUND)
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
      endif()
    endforeach()
  endif()
else()
  message(STATUS "Tests disabled - GTest not found")
endif()

# ROS2配置
if(USE_ROS2_MESSAGES)
  ament_export_include_directories(include)
  ament_export_libraries(trajectory_interpolator_core)
  ament_export_dependencies(
    rclcpp 
    trajectory_msgs 
    moveit_msgs 
    builtin_interfaces
  )
  ament_package()
endif()
