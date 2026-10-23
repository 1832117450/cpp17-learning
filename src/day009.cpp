
#include <iostream>
#include <cmath>
#include <iomanip>

struct VehicleState {
    double x;
    double y;
    double yaw;
    double speed;
};

// 使用 Euler 积分计算下一时刻车辆状态
void step(VehicleState& state,
          double steering,
          double acceleration,
          double dt,
          double wheelbase) {

    double dx = state.speed * std::cos(state.yaw);
    double dy = state.speed * std::sin(state.yaw);
    double dyaw = state.speed / wheelbase * std::tan(steering);

    state.x += dx * dt;
    state.y += dy * dt;
    state.yaw += dyaw * dt;
    state.speed += acceleration * dt;
}

int main() {
    const double dt = 0.1;
    const double wheelbase = 2.8;

    VehicleState car{0.0, 0.0, 0.0, 5.0};

    std::cout << std::fixed << std::setprecision(3);

    // 实验一：直线行驶
    for (int i = 0; i < 20; ++i) {
        step(car, 0.0, 0.0, dt, wheelbase);
    }

    std::cout << "Straight:\n";
    std::cout << "x=" << car.x
              << ", y=" << car.y
              << ", yaw=" << car.yaw
              << ", v=" << car.speed << "\n";

    // 实验二：恒定转角行驶
    car = {0.0, 0.0, 0.0, 5.0};

    for (int i = 0; i < 20; ++i) {
        step(car, 0.2, 0.0, dt, wheelbase);
    }

    std::cout << "Turning:\n";
    std::cout << "x=" << car.x
              << ", y=" << car.y
              << ", yaw=" << car.yaw
              << ", v=" << car.speed << "\n";

    return 0;
}
