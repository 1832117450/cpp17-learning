#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cstddef>

struct VehicleState {
    double x;
    double y;
    double yaw;
    double v;
};

struct ControlCommand {
    double acceleration;
    double steering;
};

struct PathPoint {
    double x;
    double y;
};

// ==============================
// 1. PID 纵向速度控制器
// ==============================
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    double update(double target, double current, double dt) {
        double error = target - current;

        integral_ += error * dt;

        double derivative = first_ ? 0.0 :
            (error - prev_error_) / dt;

        prev_error_ = error;
        first_ = false;

        return kp_ * error
             + ki_ * integral_
             + kd_ * derivative;
    }

private:
    double kp_;
    double ki_;
    double kd_;

    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_ = true;
};

// ==============================
// 2. 运动学自行车模型
// ==============================
void stepVehicle(
    VehicleState& car,
    const ControlCommand& command,
    double dt,
    double wheelbase
) {
    double a = std::clamp(
        command.acceleration, -3.0, 2.0
    );

    double delta = std::clamp(
        command.steering, -0.5, 0.5
    );

    double dx = car.v * std::cos(car.yaw) * dt;
    double dy = car.v * std::sin(car.yaw) * dt;

    double dyaw =
        car.v / wheelbase * std::tan(delta) * dt;

    car.x += dx;
    car.y += dy;
    car.yaw += dyaw;
    car.v = std::max(0.0, car.v + a * dt);
}

// ==============================
// 3. 生成 S 形参考轨迹
// ==============================
std::vector<PathPoint> generatePath() {
    std::vector<PathPoint> path;

    for (int i = 0; i <= 400; ++i) {
        double x = i * 0.5;
        double y = 2.0 * std::sin(x / 15.0);

        path.push_back({x, y});
    }

    return path;
}

// ==============================
// 4. 寻找最近的参考轨迹点
// ==============================
std::size_t findNearestPoint(
    const VehicleState& car,
    const std::vector<PathPoint>& path
) {
    std::size_t nearest = 0;

    double min_dist =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < path.size(); ++i) {

        double dx = path[i].x - car.x;
        double dy = path[i].y - car.y;

        double dist = std::hypot(dx, dy);

        if (dist < min_dist) {
            min_dist = dist;
            nearest = i;
        }
    }

    return nearest;
}

// ==============================
// 5. 寻找 Pure Pursuit 前视目标点
// ==============================
PathPoint findTargetPoint(
    const std::vector<PathPoint>& path,
    std::size_t nearest_index,
    double lookahead
) {
    std::size_t index = nearest_index;
    double accumulated = 0.0;

    while (
        index + 1 < path.size() &&
        accumulated < lookahead
    ) {
        accumulated += std::hypot(
            path[index + 1].x - path[index].x,
            path[index + 1].y - path[index].y
        );

        ++index;
    }

    return path[index];
}

// ==============================
// 6. Pure Pursuit 横向控制器
// ==============================
double purePursuit(
    const VehicleState& car,
    const PathPoint& target,
    double wheelbase
) {
    double dx = target.x - car.x;
    double dy = target.y - car.y;

    double ld = std::hypot(dx, dy);

    if (ld < 1e-6) {
        return 0.0;
    }

    double alpha =
        std::atan2(dy, dx) - car.yaw;

    double delta = std::atan2(
        2.0 * wheelbase * std::sin(alpha),
        ld
    );

    return delta;
}

// ==============================
// 7. 主程序：PID + Pure Pursuit
// ==============================
int main() {

    // 仿真参数
    const double dt = 0.1;
    const double wheelbase = 2.8;
    const double target_speed = 8.0;
    const double lookahead = 4.0;
    const int steps = 180;

    // 初始车辆状态
    VehicleState car{0.0, -1.0, 0.0, 4.0};

    // 创建 PID 控制器
    PIDController pid(0.6, 0.1, 0.05);

    // 生成参考轨迹
    auto path = generatePath();

    // 保存实验数据
    std::ofstream file(
        "results/day020_pure_pursuit.csv"
    );

    if (!file) {
        std::cerr << "Cannot open CSV file\n";
        return 1;
    }

    file << "t,x,y,yaw,v,ref_x,ref_y,"
            "target_x,target_y,error,a,delta\n";

    file << std::fixed << std::setprecision(6);

    double sum_squared_error = 0.0;
    double max_error = 0.0;
    double sum_squared_speed_error = 0.0;

    // ==========================
    // 闭环控制循环
    // ==========================
    for (int i = 0; i < steps; ++i) {

        // 第一步：寻找最近参考点
        std::size_t nearest_index =
            findNearestPoint(car, path);

        PathPoint nearest = path[nearest_index];

        // 第二步：计算轨迹跟踪误差
        double error = std::hypot(
            car.x - nearest.x,
            car.y - nearest.y
        );

        sum_squared_error += error * error;
        max_error = std::max(max_error, error);

        // 第三步：计算速度误差
        double speed_error = target_speed - car.v;

        sum_squared_speed_error +=
            speed_error * speed_error;

        // 第四步：PID 纵向控制
        double a = pid.update(
            target_speed, car.v, dt
        );

        a = std::clamp(a, -3.0, 2.0);

        // 第五步：寻找前视目标点
        PathPoint target = findTargetPoint(
            path, nearest_index, lookahead
        );

        // 第六步：Pure Pursuit 横向控制
        double delta = purePursuit(
            car, target, wheelbase
        );

        delta = std::clamp(delta, -0.5, 0.5);

        // 第七步：组合控制指令
        ControlCommand command{a, delta};

        // 第八步：记录当前车辆状态
        file << i * dt << ","
             << car.x << ","
             << car.y << ","
             << car.yaw << ","
             << car.v << ","
             << nearest.x << ","
             << nearest.y << ","
             << target.x << ","
             << target.y << ","
             << error << ","
             << a << ","
             << delta << "\n";

        // 第九步：更新车辆运动状态
        stepVehicle(
            car, command, dt, wheelbase
        );
    }

    file.close();

    // ==========================
    // 计算最终评估指标
    // ==========================
    double tracking_rmse = std::sqrt(
        sum_squared_error / steps
    );

    double speed_rmse = std::sqrt(
        sum_squared_speed_error / steps
    );

    std::cout << std::fixed << std::setprecision(4);

    std::cout << "Pure Pursuit + PID\n";

    std::cout << "Final x: "
              << car.x << " m\n";

    std::cout << "Final y: "
              << car.y << " m\n";

    std::cout << "Final speed: "
              << car.v << " m/s\n";

    std::cout << "Tracking RMSE: "
              << tracking_rmse << " m\n";

    std::cout << "Max error: "
              << max_error << " m\n";

    std::cout << "Speed RMSE: "
              << speed_rmse << " m/s\n";

    std::cout << "CSV saved: "
              << "results/day020_pure_pursuit.csv\n";

    return 0;
}
