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

## 编译方法

mkdir -p build
g++ -std=c++17 -Wall -Wextra src/main.cpp -o build/hello

## 运行方法

./build/hello

## 预期输出

Hello, C++17!
