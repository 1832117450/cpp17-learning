
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <array>

constexpr double PI = 3.14159265358979323846;

struct VehicleState {
    double x;
    double y;
    double yaw;
    double v;
};

// 使用指定时间步长模拟 3 秒
VehicleState simulate(double dt) {

    VehicleState car{0.0, 0.0, 0.0, 2.0};

    const double L = 2.7;
    const double T = 3.0;

    // 根据时间步长确定更新次数
    int steps = static_cast<int>(std::lround(T / dt));

    // 控制输入限幅
    double steering_deg = std::clamp(40.0, -20.0, 20.0);
    double acceleration = std::clamp(3.0, -1.0, 1.0);

    double steering = steering_deg * PI / 180.0;

    for (int i = 0; i < steps; ++i) {

        // 保存当前时刻的速度和航向
        double v_old = car.v;
        double yaw_old = car.yaw;

        // Euler 积分
        car.x += v_old * std::cos(yaw_old) * dt;
        car.y += v_old * std::sin(yaw_old) * dt;
        car.yaw += v_old / L * std::tan(steering) * dt;
        car.v += acceleration * dt;
    }

    return car;
}

int main() {

    const std::array<double, 3> dts = {0.1, 0.01, 0.001};
    const VehicleState reference = simulate(0.001);

    std::cout << std::fixed << std::setprecision(6);

    for (double dt : dts) {

        VehicleState car = simulate(dt);

        // 与最小时间步长结果比较位置差异
        double error = std::hypot(
            car.x - reference.x,
            car.y - reference.y
        );

        // 多次重复运行，以便测量计算耗时
        const int repeats = 1000;
        double checksum = 0.0;

        auto start = std::chrono::steady_clock::now();

        for (int j = 0; j < repeats; ++j) {
            VehicleState temp = simulate(dt);
            checksum += temp.x;
        }

        auto end = std::chrono::steady_clock::now();

        double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                end - start
            ).count();

        std::cout << "------------------------\n";
        std::cout << "dt = " << dt << " s\n";
        std::cout << "步数 = " << std::lround(3.0 / dt) << "\n";
        std::cout << "x = " << car.x << "\n";
        std::cout << "y = " << car.y << "\n";
        std::cout << "yaw = " << car.yaw << "\n";
        std::cout << "v = " << car.v << "\n";
        std::cout << "位置差异 = " << error << " m\n";
        std::cout << "1000次运行耗时 = " << elapsed_ms << " ms\n";
        std::cout << "checksum = " << checksum << "\n";
    }

    return 0;
}
