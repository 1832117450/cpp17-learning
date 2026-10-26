
#include "vehicle_model.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

constexpr double PI = 3.14159265358979323846;

// =====================================================
// 辅助函数：比较实际值与期望值
// =====================================================
void expectNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& name
) {
    if (!std::isfinite(actual) ||
        std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(name + " failed");
    }
}

// =====================================================
// 辅助函数：检查车辆状态是否被改变
// =====================================================
void expectSameState(
    const VehicleState& actual,
    const VehicleState& expected,
    const std::string& name
) {
    expectNear(actual.x, expected.x, 0.0, name + " x");
    expectNear(actual.y, expected.y, 0.0, name + " y");
    expectNear(actual.yaw, expected.yaw, 0.0, name + " yaw");
    expectNear(actual.v, expected.v, 0.0, name + " v");
}

// =====================================================
// 测试一：零速度
// =====================================================
void testZeroSpeed() {

    VehicleState car{0.0, 0.0, 0.0, 0.0};

    step(car, 20.0, 0.0, 0.1);

    expectNear(car.x, 0.0, 1e-9, "ZeroSpeed x");
    expectNear(car.y, 0.0, 1e-9, "ZeroSpeed y");
    expectNear(car.yaw, 0.0, 1e-9, "ZeroSpeed yaw");
    expectNear(car.v, 0.0, 1e-9, "ZeroSpeed v");

    std::cout << "[PASS] Zero speed test\n";
}

// =====================================================
// 测试二：直线行驶
// =====================================================
void testStraightLine() {

    VehicleState car{0.0, 0.0, 0.0, 2.0};

    const double dt = 0.1;

    for (int i = 0; i < 10; ++i) {
        step(car, 0.0, 0.0, dt);
    }

    expectNear(car.x, 2.0, 1e-9, "StraightLine x");
    expectNear(car.y, 0.0, 1e-9, "StraightLine y");
    expectNear(car.yaw, 0.0, 1e-9, "StraightLine yaw");
    expectNear(car.v, 2.0, 1e-9, "StraightLine v");

    std::cout << "[PASS] Straight line test\n";
}

// =====================================================
// 测试三：圆周运动
// =====================================================
void testCircularMotion() {

    VehicleState car{0.0, 0.0, 0.0, 2.0};

    const double L = 2.7;
    const double steering_deg = 10.0;
    const double steering = steering_deg * PI / 180.0;

    const double dt = 0.01;
    const double T = 5.0;
    const int steps = 500;

    // 理论转弯半径
    const double R = L / std::tan(steering);

    // 理论角速度
    const double omega = car.v / R;

    // 理论航向角
    const double expected_yaw = omega * T;

    // 理论轨迹终点
    const double expected_x =
        R * std::sin(expected_yaw);

    const double expected_y =
        R * (1.0 - std::cos(expected_yaw));

    for (int i = 0; i < steps; ++i) {
        step(car, steering_deg, 0.0, dt);
    }

    std::cout << "\n--- Circular Motion Results ---\n";

    std::cout << "Theory radius = " << R << " m\n";

    std::cout << "Theory x = " << expected_x
              << ", Actual x = " << car.x << "\n";

    std::cout << "Theory y = " << expected_y
              << ", Actual y = " << car.y << "\n";

    std::cout << "Theory yaw = " << expected_yaw
              << ", Actual yaw = " << car.yaw << "\n";

    expectNear(car.x, expected_x, 0.02, "Circular x");
    expectNear(car.y, expected_y, 0.02, "Circular y");
    expectNear(car.yaw, expected_yaw, 1e-6, "Circular yaw");
    expectNear(car.v, 2.0, 1e-9, "Circular speed");

    std::cout << "[PASS] Circular motion test\n";
}

// =====================================================
// 测试四：非法输入
// =====================================================
void testInvalidInputs() {

    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    const double inf =
        std::numeric_limits<double>::infinity();

    // 检查非法输入是否被拒绝
    auto checkRejected = [&](
        double steering,
        double acceleration,
        double dt,
        const std::string& name
    ) {

        VehicleState car{1.0, 2.0, 0.3, 2.0};
        const VehicleState before = car;

        bool rejected = false;

        try {
            step(car, steering, acceleration, dt);
        }
        catch (const std::invalid_argument&) {
            rejected = true;
        }

        if (!rejected) {
            throw std::runtime_error(
                name + " was not rejected"
            );
        }

        // 非法输入不应修改原有状态
        expectSameState(car, before, name);
    };

    checkRejected(0.0, 0.0, 0.0, "Zero dt");
    checkRejected(0.0, 0.0, -0.1, "Negative dt");
    checkRejected(0.0, 0.0, nan, "NaN dt");
    checkRejected(0.0, 0.0, inf, "Infinite dt");
    checkRejected(nan, 0.0, 0.1, "NaN steering");
    checkRejected(0.0, inf, 0.1, "Infinite acceleration");

    // 测试非法车辆状态
    VehicleState invalidCar{0.0, 0.0, 0.0, nan};

    bool rejected = false;

    try {
        step(invalidCar, 0.0, 0.0, 0.1);
    }
    catch (const std::invalid_argument&) {
        rejected = true;
    }

    if (!rejected) {
        throw std::runtime_error(
            "Invalid vehicle state was not rejected"
        );
    }

    std::cout << "[PASS] Invalid input test\n";
}

// =====================================================
// 测试五：控制限幅
// =====================================================
void testControlLimits() {

    // 超限输入
    VehicleState car1{0.0, 0.0, 0.0, 2.0};

    step(car1, 40.0, 3.0, 0.1);

    // 限幅后的合法输入
    VehicleState car2{0.0, 0.0, 0.0, 2.0};

    step(car2, 20.0, 1.0, 0.1);

    // 两者结果应相同
    expectSameState(car1, car2, "Positive control limits");

    // 检查负方向限幅
    VehicleState car3{0.0, 0.0, 0.0, 2.0};
    VehicleState car4{0.0, 0.0, 0.0, 2.0};

    step(car3, -40.0, -3.0, 0.1);
    step(car4, -20.0, -1.0, 0.1);

    expectSameState(car3, car4, "Negative control limits");

    std::cout << "[PASS] Control limits test\n";
}

// =====================================================
// 主函数：统一运行所有测试
// =====================================================
int main() {

    try {

        std::cout
            << "=== Day 013 Vehicle Model Tests ===\n\n";

        testZeroSpeed();

        testStraightLine();

        testCircularMotion();

        testInvalidInputs();

        testControlLimits();

        std::cout << "\nAll tests passed!\n";

    }
    catch (const std::exception& e) {

        std::cerr << "[FAIL] "
                  << e.what() << "\n";

        return 1;
    }

    return 0;
}
