#pragma once

// 车辆状态
struct VehicleState {
    double x;
    double y;
    double speed;
};

// 控制命令
struct ControlCommand {
    double acceleration;
    double steering;
};
