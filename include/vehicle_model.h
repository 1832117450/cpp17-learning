
#pragma once

// 车辆状态
struct VehicleState {
    double x;
    double y;
    double yaw;
    double v;
};

// 声明车辆状态更新函数
void step(
    VehicleState& car,
    double steering_deg,
    double acceleration,
    double dt
);
