#!/usr/bin/env bash
set -euo pipefail

# 进入项目根目录
cd "$(dirname "$0")/.."

mkdir -p build results

# 编译车辆模型和测试程序
g++ -std=c++17 -Wall -Wextra -Iinclude \
    src/vehicle_model.cpp \
    tests/day013_tests.cpp \
    -o build/day013_tests

# 运行测试并保存结果
./build/day013_tests | tee results/day013.txt
