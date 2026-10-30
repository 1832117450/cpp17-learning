
#include <iostream>
#include <Eigen/Dense>

int main() {
    // 1. 时间步长
    const double dt = 0.1;

    // 2. 当前状态 [位置, 速度]
    Eigen::Vector2d x;
    x << 0.0, 10.0;

    // 3. 控制输入：加速度
    const double u = 2.0;

    // 4. 状态转移矩阵 A
    Eigen::Matrix2d A;
    A << 1.0, dt,
         0.0, 1.0;

    // 5. 控制输入矩阵 B
    Eigen::Vector2d B;
    B << 0.0, dt;

    // 6. 计算下一时刻状态
    Eigen::Vector2d x_next = A * x + B * u;

    // 7. 输出结果
    std::cout << "Current state:\n"
              << x << "\n";

    std::cout << "Matrix A:\n"
              << A << "\n";

    std::cout << "Matrix B:\n"
              << B << "\n";

    std::cout << "Next state:\n"
              << x_next << "\n";

    return 0;
}
