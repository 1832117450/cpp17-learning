
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <string>

constexpr double PI = 3.14159265358979323846;
constexpr double WHEELBASE = 2.7;

// 1. 定义车辆状态
struct VehicleState {
    double x;
    double y;
    double yaw;  // 航向角，单位 rad
    double v;    // 车速，单位 m/s
};

// 2. 定义跟踪误差
struct TrackingError {
    double headingError;
    double crossTrackError;
};

// 3. 角度归一化
double normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// 4. 计算直线路径的跟踪误差
TrackingError calculateError(const VehicleState& car) {
    // 车辆前轴中心坐标
    double frontY = car.y + WHEELBASE * std::sin(car.yaw);

    // 参考路径：y = 0，航向角 = 0
    double headingError = normalizeAngle(0.0 - car.yaw);

    // 路径位于前轴左侧时为正
    double crossTrackError = 0.0 - frontY;

    return {headingError, crossTrackError};
}

// 5. Stanley 控制器
double stanleyControl(
    double headingError,
    double crossTrackError,
    double velocity,
    double k
) {
    const double epsilon = 0.1;
    const double maxSteer = 0.6;

    // 航向误差反馈
    double headingTerm = normalizeAngle(headingError);

    // 横向误差反馈
    double lateralTerm =
        std::atan2(k * crossTrackError,
                   std::max(0.0, velocity) + epsilon);

    // 总转向角
    double delta = headingTerm + lateralTerm;

    // 限制最大转向角
    return std::clamp(delta, -maxSteer, maxSteer);
}

// 6. 测试不同车辆状态
void testVehicle(const std::string& name,
                 const VehicleState& car) {

    TrackingError error = calculateError(car);

    double delta = stanleyControl(
        error.headingError,
        error.crossTrackError,
        car.v,
        1.0
    );

    std::cout << name << "\n";
    std::cout << "横向误差: "
              << error.crossTrackError << " m\n";
    std::cout << "航向误差: "
              << error.headingError * 180.0 / PI << " deg\n";
    std::cout << "转向角: "
              << delta * 180.0 / PI << " deg\n\n";
}

int main() {
    VehicleState car1{0.0, -1.0, 0.0, 5.0};
    VehicleState car2{0.0,  1.0, 0.0, 5.0};
    VehicleState car3{0.0,  0.0, PI / 18.0, 5.0};
    VehicleState car4{0.0, -1.0, 0.0, 0.0};

    testVehicle("Case 1: Path on left", car1);
    testVehicle("Case 2: Path on right", car2);
    testVehicle("Case 3: Heading deviation", car3);
    testVehicle("Case 4: Zero velocity", car4);

    // 基础方向与限幅检查
    auto e1 = calculateError(car1);
    auto e2 = calculateError(car2);
    auto e4 = calculateError(car4);

    assert(stanleyControl(e1.headingError,
                          e1.crossTrackError, car1.v, 1.0) > 0);

    assert(stanleyControl(e2.headingError,
                          e2.crossTrackError, car2.v, 1.0) < 0);

    assert(std::abs(stanleyControl(e4.headingError,
                                   e4.crossTrackError,
                                   car4.v, 1.0)) <= 0.6);

    std::cout << "All checks passed.\n";
    return 0;
}
