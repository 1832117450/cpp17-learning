
#include "vehicle_model.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

constexpr double PI = 3.14159265358979323846;

void step(
    VehicleState& car,
    double steering_deg,
    double acceleration,
    double dt
) {
    // 检查非法输入
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(steering_deg) ||
        !std::isfinite(acceleration) ||
        !std::isfinite(car.x) ||
        !std::isfinite(car.y) ||
        !std::isfinite(car.yaw) ||
        !std::isfinite(car.v)) {
        throw std::invalid_argument("Invalid vehicle input");
    }

    // 车辆参数
    const double L = 2.7;

    // 控制限幅
    steering_deg = std::clamp(
        steering_deg, -20.0, 20.0
    );

    acceleration = std::clamp(
        acceleration, -1.0, 1.0
    );

    // 角度转换
    double steering = steering_deg * PI / 180.0;

    // 保留原始状态，进行显式 Euler 积分
    double v_old = car.v;
    double yaw_old = car.yaw;

    car.x += v_old * std::cos(yaw_old) * dt;
    car.y += v_old * std::sin(yaw_old) * dt;
    car.yaw += v_old / L * std::tan(steering) * dt;
    car.v += acceleration * dt;
}
