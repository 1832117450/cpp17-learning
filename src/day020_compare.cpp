#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cstddef>
#include <string>
#include <cstdlib>

// ==============================
// 1. 基本数据结构
// ==============================
struct VehicleState {
    double x, y, yaw, v;
};

struct ControlCommand {
    double acceleration, steering;
};

struct PathPoint {
    double x, y;
};

struct Metrics {
    double tracking_rmse;
    double max_error;
    double speed_rmse;
    double final_x, final_y, final_speed;
};

// ==============================
// 2. PID 纵向控制器
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
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_ = true;
};

// ==============================
// 3. 运动学自行车模型
// ==============================
void stepVehicle(
    VehicleState& car,
    const ControlCommand& u,
    double dt,
    double L
) {
    double a = std::clamp(u.acceleration, -3.0, 2.0);
    double delta = std::clamp(u.steering, -0.5, 0.5);

    double dx = car.v * std::cos(car.yaw) * dt;
    double dy = car.v * std::sin(car.yaw) * dt;
    double dyaw = car.v / L * std::tan(delta) * dt;

    car.x += dx;
    car.y += dy;
    car.yaw += dyaw;
    car.v = std::max(0.0, car.v + a * dt);
}

// ==============================
// 4. 生成 S 形参考路径
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
// 5. 寻找最近参考点
// ==============================
std::size_t findNearestPoint(
    double x,
    double y,
    const std::vector<PathPoint>& path
) {
    std::size_t nearest = 0;
    double best_sq =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].x - x;
        double dy = path[i].y - y;
        double sq = dx * dx + dy * dy;

        if (sq < best_sq) {
            best_sq = sq;
            nearest = i;
        }
    }

    return nearest;
}

// ==============================
// 6. Pure Pursuit 前视目标点
// ==============================
PathPoint findTargetPoint(
    const std::vector<PathPoint>& path,
    std::size_t index,
    double lookahead
) {
    double arc_length = 0.0;

    while (index + 1 < path.size() &&
           arc_length < lookahead) {

        arc_length += std::hypot(
            path[index + 1].x - path[index].x,
            path[index + 1].y - path[index].y
        );

        ++index;
    }

    return path[index];
}

// ==============================
// 7. Pure Pursuit 横向控制器
// ==============================
double purePursuit(
    const VehicleState& car,
    const PathPoint& target,
    double L
) {
    double dx = target.x - car.x;
    double dy = target.y - car.y;
    double ld = std::hypot(dx, dy);

    if (ld < 1e-6) return 0.0;

    double alpha = std::atan2(dy, dx) - car.yaw;

    return std::atan2(
        2.0 * L * std::sin(alpha), ld
    );
}

// ==============================
// 8. 航向角归一化
// ==============================
double normalizeAngle(double angle) {
    return std::atan2(
        std::sin(angle), std::cos(angle)
    );
}

// ==============================
// 9. Stanley 横向控制器
// ==============================
double stanley(
    const VehicleState& car,
    const std::vector<PathPoint>& path,
    double L,
    double gain
) {
    // 计算车辆前轴位置
    double fx = car.x + L * std::cos(car.yaw);
    double fy = car.y + L * std::sin(car.yaw);

    // 找到前轴对应的最近参考点
    std::size_t idx = findNearestPoint(fx, fy, path);

    std::size_t prev = (idx == 0) ? 0 : idx - 1;
    std::size_t next =
        (idx + 1 == path.size()) ? idx : idx + 1;

    // 计算参考路径方向
    double tx = path[next].x - path[prev].x;
    double ty = path[next].y - path[prev].y;

    double path_yaw = std::atan2(ty, tx);

    // 带符号横向误差
    // 参考线位于前轴左侧时为正
    double signed_error =
        std::cos(path_yaw) * (path[idx].y - fy)
      - std::sin(path_yaw) * (path[idx].x - fx);

    // 航向误差
    double heading_error = normalizeAngle(
        path_yaw - car.yaw
    );

    // Stanley 控制律
    double delta = heading_error
        + std::atan2(
            gain * signed_error,
            car.v + 0.5
        );

    return delta;
}

// ==============================
// 10. 统一闭环仿真实验
// ==============================
Metrics runExperiment(
    bool use_stanley,
    const std::vector<PathPoint>& path,
    const std::string& csv_file
) {
    const double dt = 0.1;
    const double L = 2.8;
    const double target_speed = 8.0;
    const double lookahead = 4.0;
    const double stanley_gain = 2.0;
    const int steps = 180;

    // 两组实验使用完全相同的初始状态
    VehicleState car{0.0, -1.0, 0.0, 4.0};
    PIDController pid(0.6, 0.1, 0.05);

    std::ofstream file(csv_file);

    if (!file) {
        std::cerr << "Cannot open " << csv_file << "\n";
        std::exit(1);
    }

    file << "t,x,y,yaw,v,ref_x,ref_y,"
            "error,a,delta\n";

    file << std::fixed << std::setprecision(6);

    double sum_sq_error = 0.0;
    double max_error = 0.0;
    double sum_sq_speed_error = 0.0;

    // 闭环控制
    for (int i = 0; i < steps; ++i) {

        // 最近参考点
        std::size_t idx = findNearestPoint(
            car.x, car.y, path
        );

        PathPoint nearest = path[idx];

        // 统一跟踪误差指标
        double error = std::hypot(
            car.x - nearest.x,
            car.y - nearest.y
        );

        sum_sq_error += error * error;
        max_error = std::max(max_error, error);

        // 速度误差
        double speed_error = target_speed - car.v;
        sum_sq_speed_error += speed_error * speed_error;

        // PID 纵向控制
        double a = std::clamp(
            pid.update(target_speed, car.v, dt),
            -3.0, 2.0
        );

        // 根据实验选择横向控制器
        double delta;

        if (use_stanley) {
            delta = stanley(
                car, path, L, stanley_gain
            );
        } else {
            PathPoint target = findTargetPoint(
                path, idx, lookahead
            );

            delta = purePursuit(car, target, L);
        }

        delta = std::clamp(delta, -0.5, 0.5);

        // 联合控制
        ControlCommand command{a, delta};

        // 保存当前状态
        file << i * dt << ","
             << car.x << ","
             << car.y << ","
             << car.yaw << ","
             << car.v << ","
             << nearest.x << ","
             << nearest.y << ","
             << error << ","
             << a << ","
             << delta << "\n";

        // 车辆状态更新
        stepVehicle(car, command, dt, L);
    }

    file.close();

    return {
        std::sqrt(sum_sq_error / steps),
        max_error,
        std::sqrt(sum_sq_speed_error / steps),
        car.x,
        car.y,
        car.v
    };
}

// ==============================
// 11. 输出实验结果
// ==============================
void printMetrics(
    const std::string& name,
    const Metrics& m
) {
    std::cout << name << "\n";

    std::cout << "  Final x: "
              << m.final_x << " m\n";

    std::cout << "  Final y: "
              << m.final_y << " m\n";

    std::cout << "  Final speed: "
              << m.final_speed << " m/s\n";

    std::cout << "  Tracking RMSE: "
              << m.tracking_rmse << " m\n";

    std::cout << "  Max error: "
              << m.max_error << " m\n";

    std::cout << "  Speed RMSE: "
              << m.speed_rmse << " m/s\n";
}

// ==============================
// 12. 主程序
// ==============================
int main() {
    auto path = generatePath();

    // 实验 A: PID + Pure Pursuit
    Metrics pp = runExperiment(
        false, path,
        "results/day020_pp_compare.csv"
    );

    // 实验 B: PID + Stanley
    Metrics st = runExperiment(
        true, path,
        "results/day020_stanley.csv"
    );

    std::cout << std::fixed << std::setprecision(4);

    printMetrics("PID + Pure Pursuit", pp);
    printMetrics("PID + Stanley", st);

    std::cout << "\n";
    std::cout << "Comparison completed.\n";
    std::cout << "Saved: results/day020_pp_compare.csv\n";
    std::cout << "Saved: results/day020_stanley.csv\n";

    return 0;
}
