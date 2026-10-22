cmake_minimum_required(VERSION 3.16)

project(cpp17_learning LANGUAGES CXX)

# 使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 GoogleTest 和线程依赖
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# 创建测试可执行程序
add_executable(run_tests
    tests/test_distance.cpp
    tests/test_angle.cpp
    tests/test_trajectory.cpp
)

# 链接测试库
target_link_libraries(run_tests PRIVATE
    GTest::gtest_main
    Threads::Threads
)

# 启用 CMake 测试功能
enable_testing()

# 注册测试
add_test(NAME unit_tests COMMAND run_tests)

# Day 006: 添加主程序编译目标
add_executable(hello
    src/main.cpp
)

# Day 006: 可复用轨迹模块
add_library(planning_core
    src/trajectory.cpp
)

target_include_directories(planning_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Day 006: 工程集成示例
add_executable(day06
    src/day06.cpp
)

target_link_libraries(day06 PRIVATE
    planning_core
)

# Day 006: 让测试程序链接轨迹核心库
target_link_libraries(run_tests PRIVATE planning_core)
