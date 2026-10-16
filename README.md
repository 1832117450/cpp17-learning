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
