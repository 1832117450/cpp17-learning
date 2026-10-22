# C++17 Learning

我的 C++17 编程学习工程。

## 开发环境

- 操作系统：Ubuntu 20.04
- 编译器：G++ 9.4.0
- C++ 标准：C++17
- 编辑器：VS Code Remote SSH

## 项目结构

- src/：C++ 源代码
- notes/：学习笔记
- results/：运行结果
- build/：编译生成的文件

## 学习进度

- 第001天：建立 C++17 学习工程
- 第002天：引用与 const

## Day 001

### 编译

g++ -std=c++17 -Wall -Wextra src/main.cpp -o build/hello

### 运行

./build/hello

### 输出

Hello, C++17!

## Day 002

### 编译

g++ -std=c++17 -Wall -Wextra src/day02.cpp -o build/day02

### 运行

./build/day02

### 学习内容

- 值传递
- 引用传递
- 指针传递
- const 引用
- VehicleState 车辆状态结构体
- 编译警告与编译错误

## Day 003：STL 与轨迹容器

### 学习内容
- std::vector：动态存储轨迹点
- std::array：保存固定长度的二维坐标
- std::optional：处理不存在的轨迹点
- const auto&：通过常量引用遍历容器
- struct TrajectoryPoint：定义轨迹点数据结构

### 完成任务
- 定义包含 x、y、speed 的轨迹点结构体
- 使用 vector 保存并打印 10 个轨迹点
- 使用 optional 检查非法索引
- 使用 empty() 检查空轨迹

### 编译与运行

g++ -std=c++17 -Wall -Wextra src/day03.cpp -o build/day03

./build/day03

### 学习记录
- 代码：src/day03.cpp
- 笔记：notes/day003.md
- 运行结果：results/day003.txt

### 自动驾驶应用
使用 vector 保存规划轨迹，使用 optional 处理无效轨迹点，为后续车辆模型、轨迹规划和控制算法打基础。


## Day 006：模块化工程

本阶段将车辆状态和轨迹容器整理为可复用模块。

### 工程结构

- include/：头文件
- src/：源代码
- tests/：单元测试
- notes/：学习笔记
- results/：运行结果
- build/：编译输出

### 编译

```bash
cmake -S . -B build
cmake --build build

```

### 运行 Day 006

    ./build/day06

### 运行测试

    ./build/run_tests

也可以运行：

    cd build
    ctest --output-on-failure

### 核心模块

- VehicleState：车辆状态
- ControlCommand：控制命令
- Trajectory：轨迹点管理
- planning_core：可复用的 C++ 静态库

