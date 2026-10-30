
#include <iostream>
#include <iomanip>

// 简化的一步最优控制示例
// e_next = e + u
// J = q * e_next^2 + r * u^2

void testWeights(double q, double r, double error) {
    // 由一步代价最小化得到反馈增益
    double K = q / (q + r);

    // 最优控制输入
    double u = -K * error;

    // 更新误差
    double next_error = error + u;

    std::cout << "Q = " << q
              << ", R = " << r << "\n";

    std::cout << "K = " << K << "\n";
    std::cout << "u = " << u << "\n";
    std::cout << "Next error = "
              << next_error << "\n\n";
}

int main() {
    // 实际速度 13，目标速度 15
    double current_speed = 13.0;
    double target_speed = 15.0;

    double error = current_speed - target_speed;

    std::cout << std::fixed
              << std::setprecision(3);

    testWeights(1.0, 1.0, error);
    testWeights(9.0, 1.0, error);
    testWeights(1.0, 9.0, error);

    return 0;
}
