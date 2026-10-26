
#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>

constexpr double PI = 3.14159265358979323846;

struct VehicleState {
    double x;
    double y;
    double yaw;
    double v;
};

void step(VehicleState& car,
          double steering_deg,
          double acceleration,
          double dt) {

    // 1. 设定车辆控制边界
    const double max_steering_deg = 20.0;
    const double max_acceleration = 1.0;

    // 2. 限制控制输入
    steering_deg = std::clamp(
        steering_deg,
        -max_steering_deg,
        max_steering_deg
    );

    acceleration = std::clamp(
        acceleration,
        -max_acceleration,
        max_acceleration
    );

    // 3. 角度转换：度 -> 弧度
    double steering = steering_deg * PI / 180.0;

    // 4. 车辆轴距
    double L = 2.7;

    // 5. Euler 方法更新车辆状态
    car.x += car.v * std::cos(car.yaw) * dt;
    car.y += car.v * std::sin(car.yaw) * dt;
    car.yaw += car.v / L * std::tan(steering) * dt;
    car.v += acceleration * dt;

    std::cout << "实际转向角: " << steering_deg << " deg\n";
    std::cout << "实际加速度: " << acceleration << " m/s^2\n";
}

int main() {

    VehicleState car{0.0, 0.0, 0.0, 2.0};

    std::cout << std::fixed << std::setprecision(4);

    // 期望转向 40°，期望加速度 3 m/s^2
    step(car, 40.0, 3.0, 0.1);

    std::cout << "x = " << car.x << "\n";
    std::cout << "y = " << car.y << "\n";
    std::cout << "yaw = " << car.yaw << "\n";
    std::cout << "v = " << car.v << "\n";

    return 0;
}
