#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// =====================================
// 1. 数据结构
// =====================================
struct VehicleState {
    double x, y, yaw, v;
};

struct PathPoint {
    double x, y;
};

struct Scenario {
    std::string name;
    double initial_y;
    double initial_speed;
    double target_speed;
    double lookahead;
    double amplitude;
    double scale;
    int steps;
};

struct Metrics {
    double rmse;
    double late_rmse;
    double max_error;
    double late_max;
    int steering_limit_steps;
};

// =====================================
// 2. PID 纵向控制器
// =====================================
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    double update(double target, double actual, double dt) {
        double error = target - actual;
        integral_ += error * dt;

        double derivative = first_ ? 0.0 :
            (error - previous_) / dt;

        first_ = false;
        previous_ = error;

        return kp_ * error
             + ki_ * integral_
             + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double previous_ = 0.0;
    bool first_ = true;
};

// =====================================
// 3. 生成 S 形参考路径
// =====================================
std::vector<PathPoint> makePath(
    double amplitude,
    double scale
) {
    std::vector<PathPoint> path;

    for (int i = 0; i <= 600; ++i) {
        double x = i * 0.5;
        double y = amplitude * std::sin(x / scale);
        path.push_back({x, y});
    }

    return path;
}

// =====================================
// 4. 寻找最近参考点
// =====================================
std::size_t nearestIndex(
    const VehicleState& car,
    const std::vector<PathPoint>& path
) {
    std::size_t best = 0;
    double min_sq =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < path.size(); ++i) {
        double dx = car.x - path[i].x;
        double dy = car.y - path[i].y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_sq) {
            min_sq = dist_sq;
            best = i;
        }
    }

    return best;
}

// =====================================
// 5. 计算前视目标点
// =====================================
PathPoint lookaheadTarget(
    const std::vector<PathPoint>& path,
    std::size_t index,
    double lookahead
) {
    double arc_length = 0.0;

    while (
        index + 1 < path.size() &&
        arc_length < lookahead
    ) {
        arc_length += std::hypot(
            path[index + 1].x - path[index].x,
            path[index + 1].y - path[index].y
        );

        ++index;
    }

    return path[index];
}

// =====================================
// 6. Pure Pursuit 横向控制器
// =====================================
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

    return std::atan2(
        2.0 * wheelbase * std::sin(alpha),
        ld
    );
}

// =====================================
// 7. 自行车运动学模型
// =====================================
void stepVehicle(
    VehicleState& car,
    double acceleration,
    double steering,
    double dt,
    double wheelbase
) {
    double dx =
        car.v * std::cos(car.yaw) * dt;

    double dy =
        car.v * std::sin(car.yaw) * dt;

    double dyaw =
        car.v / wheelbase *
        std::tan(steering) * dt;

    car.x += dx;
    car.y += dy;
    car.yaw += dyaw;

    car.v = std::max(
        0.0,
        car.v + acceleration * dt
    );
}

// =====================================
// 8. 执行单组闭环实验
// =====================================
Metrics runScenario(const Scenario& cfg) {

    const double dt = 0.1;
    const double wheelbase = 2.8;

    // 排除前 4 秒的初始恢复阶段
    const int warmup_steps = 40;

    auto path = makePath(
        cfg.amplitude,
        cfg.scale
    );

    VehicleState car{
        0.0,
        cfg.initial_y,
        0.0,
        cfg.initial_speed
    };

    PIDController pid(0.6, 0.1, 0.05);

    std::ofstream csv(
        "results/day021_" + cfg.name + ".csv"
    );

    if (!csv) {
        throw std::runtime_error(
            "Cannot open CSV for " + cfg.name
        );
    }

    csv << "t,x,y,ref_x,ref_y,error,v,a,delta\n";
    csv << std::fixed << std::setprecision(6);

    double error_sq_sum = 0.0;
    double late_error_sq_sum = 0.0;

    double max_error = 0.0;
    double late_max = 0.0;

    int steering_limit_steps = 0;

    for (int i = 0; i < cfg.steps; ++i) {

        // 1. 寻找最近参考点
        std::size_t nearest =
            nearestIndex(car, path);

        PathPoint reference = path[nearest];

        // 2. 计算当前位置误差
        double error = std::hypot(
            car.x - reference.x,
            car.y - reference.y
        );

        error_sq_sum += error * error;
        max_error = std::max(max_error, error);

        // 3. 单独记录 4 秒后的跟踪误差
        if (i >= warmup_steps) {
            late_error_sq_sum += error * error;
            late_max = std::max(late_max, error);
        }

        // 4. PID 纵向控制
        double a = std::clamp(
            pid.update(cfg.target_speed, car.v, dt),
            -3.0,
            2.0
        );

        // 5. 寻找 Pure Pursuit 目标点
        PathPoint target = lookaheadTarget(
            path,
            nearest,
            cfg.lookahead
        );

        // 6. Pure Pursuit 横向控制
        double raw_delta = purePursuit(
            car,
            target,
            wheelbase
        );

        // 7. 转向限幅
        double delta = std::clamp(
            raw_delta,
            -0.5,
            0.5
        );

        if (std::abs(raw_delta) >= 0.5) {
            ++steering_limit_steps;
        }

        // 8. 保存当前状态
        csv << i * dt << ","
            << car.x << ","
            << car.y << ","
            << reference.x << ","
            << reference.y << ","
            << error << ","
            << car.v << ","
            << a << ","
            << delta << "\n";

        // 9. 更新车辆状态
        stepVehicle(
            car, a, delta, dt, wheelbase
        );
    }

    return {
        std::sqrt(error_sq_sum / cfg.steps),
        std::sqrt(
            late_error_sq_sum /
            (cfg.steps - warmup_steps)
        ),
        max_error,
        late_max,
        steering_limit_steps
    };
}

// =====================================
// 9. 主程序：正常、困难、修正
// =====================================
int main() {

    std::vector<Scenario> scenarios = {

        // 正常场景：与 Day 020 保持一致
        {
            "normal",
            -1.0, 4.0, 8.0,
            4.0, 2.0, 15.0, 180
        },

        // 困难场景：速度更高、弯道更急、
        // 初始偏差更大，并使用较长前视距离
        {
            "difficult",
            -3.0, 16.0, 16.0,
            15.0, 4.0, 8.0, 140
        },

        // 修正场景：仅缩短前视距离
        {
            "adjusted",
            -3.0, 16.0, 16.0,
            4.0, 4.0, 8.0, 140
        }
    };

    std::cout << std::fixed
              << std::setprecision(4);

    std::cout
        << "Day 021: Pure Pursuit failure analysis\n";

    std::cout
        << "Teaching threshold: "
        << "post-4s RMSE <= 1.0 m\n\n";

    for (const auto& scenario : scenarios) {

        Metrics m = runScenario(scenario);

        std::cout
            << "[" << scenario.name << "]\n"
            << "  Lookahead: "
            << scenario.lookahead << " m\n"
            << "  Overall RMSE: "
            << m.rmse << " m\n"
            << "  Post-4s RMSE: "
            << m.late_rmse << " m\n"
            << "  Max error: "
            << m.max_error << " m\n"
            << "  Post-4s max error: "
            << m.late_max << " m\n"
            << "  Steering saturation steps: "
            << m.steering_limit_steps << "\n"
            << "  Tracking requirement: "
            << (
                m.late_rmse <= 1.0
                ? "MET"
                : "NOT MET"
            )
            << "\n\n";
    }

    std::cout
        << "CSV files saved in results/\n";

    return 0;
}
