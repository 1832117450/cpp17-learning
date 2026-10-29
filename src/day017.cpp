
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

constexpr double PI = 3.14159265358979323846;
constexpr double WHEELBASE = 2.8;  // 轴距 m
constexpr double SPEED = 5.0;      // 车速 m/s
constexpr double DT = 0.05;        // 时间步 s
constexpr double DURATION = 30.0;  // 仿真时长 s
constexpr double MAX_STEER = 30.0 * PI / 180.0;
constexpr double CIRCLE_RADIUS = 20.0;

// ==============================
// 1. 数据结构
// ==============================

struct Point {
    double x;
    double y;
};

struct VehicleState {
    double x;
    double y;
    double yaw;
};

struct Path {
    std::vector<Point> points;
    bool closed;
};

struct Scenario {
    std::string name;
    VehicleState initial;
    Path path;
    bool circular;
};

struct Metrics {
    double rmse;
    double max_error;
    double final_error;
    double max_steer_deg;
};

// ==============================
// 2. 生成参考路径
// ==============================

// 直线参考路径
Path make_straight_path() {
    Path path{{}, false};

    for (int i = 0; i <= 400; ++i) {
        path.points.push_back({0.5 * i, 0.0});
    }

    return path;
}

// 圆形参考路径
Path make_circle_path() {
    Path path{{}, true};

    for (int i = 0; i < 100; ++i) {
        double theta = 2.0 * PI * i / 100.0;

        path.points.push_back({
            CIRCLE_RADIUS * std::cos(theta),
            CIRCLE_RADIUS * std::sin(theta)
        });
    }

    return path;
}

// ==============================
// 3. 距离计算
// ==============================

double distance(const VehicleState& state,
                const Point& p) {

    return std::hypot(
        p.x - state.x,
        p.y - state.y
    );
}

// ==============================
// 4. 寻找 Pure Pursuit 目标点
// ==============================

Point find_target(const VehicleState& state,
                  const Path& path,
                  double lookahead) {

    if (path.points.empty()) {
        throw std::runtime_error("Path cannot be empty");
    }

    // 第一步：寻找距离车辆最近的路径点
    std::size_t nearest = 0;

    double best_distance =
        distance(state, path.points[0]);

    for (std::size_t i = 1;
         i < path.points.size(); ++i) {

        double d = distance(state, path.points[i]);

        if (d < best_distance) {
            best_distance = d;
            nearest = i;
        }
    }

    // 第二步：沿路径前进寻找目标点
    std::size_t n = path.points.size();

    for (std::size_t step = 1; step < n; ++step) {

        std::size_t i = path.closed
            ? (nearest + step) % n
            : std::min(nearest + step, n - 1);

        // 选择距离车辆达到前视距离的路径点
        if (distance(state, path.points[i]) >= lookahead) {
            return path.points[i];
        }

        if (!path.closed && i == n - 1) {
            break;
        }
    }

    // 没有找到合适的目标点时使用备用点
    return path.points[
        path.closed ? nearest : n - 1
    ];
}

// ==============================
// 5. Pure Pursuit 控制器
// ==============================

double pure_pursuit_steer(const VehicleState& state,
                          const Point& target) {

    // 车辆到目标点的相对位置
    double dx = target.x - state.x;
    double dy = target.y - state.y;

    // 目标点方向与车辆航向的夹角
    double alpha =
        std::atan2(dy, dx) - state.yaw;

    // 关键修正：
    // 使用车辆到目标点的实际欧氏距离
    // 而不是直接使用设定的 lookahead 参数
    double actual_ld = std::hypot(dx, dy);

    // 防止除零
    if (actual_ld < 1e-9) {
        return 0.0;
    }

    // Pure Pursuit 曲率公式
    double curvature =
        2.0 * std::sin(alpha) / actual_ld;

    // 曲率转换为前轮转角
    double delta =
        std::atan(WHEELBASE * curvature);

    // 转向角限幅
    delta = std::clamp(
        delta,
        -MAX_STEER,
        MAX_STEER
    );

    return delta;
}

// ==============================
// 6. 运动学自行车模型
// ==============================

VehicleState step_bicycle(VehicleState state,
                          double delta) {

    // 更新车辆位置
    state.x += SPEED * std::cos(state.yaw) * DT;

    state.y += SPEED * std::sin(state.yaw) * DT;

    // 更新车辆航向角
    state.yaw +=
        SPEED / WHEELBASE
        * std::tan(delta) * DT;

    return state;
}

// ==============================
// 7. 横向跟踪误差
// ==============================

double lateral_error(const VehicleState& state,
                     bool circular) {

    if (circular) {

        // 圆形道路：
        // 实际半径与参考半径之差的绝对值
        return std::abs(
            std::hypot(state.x, state.y)
            - CIRCLE_RADIUS
        );
    }

    // 直线道路 y = 0
    return std::abs(state.y);
}

// ==============================
// 8. 前视距离文件名转换
// ==============================

std::string ld_tag(double ld) {

    std::string tag = std::to_string(ld);

    tag = tag.substr(
        0,
        tag.find('.') + 2
    );

    std::replace(
        tag.begin(),
        tag.end(),
        '.',
        'p'
    );

    return tag;
}

// ==============================
// 9. 单组闭环仿真实验
// ==============================

Metrics run_simulation(const Scenario& scenario,
                       double lookahead) {

    // 构建输出文件名
    std::string filename =
        "results/day017_" + scenario.name
        + "_ld" + ld_tag(lookahead) + ".csv";

    std::ofstream file(filename);

    if (!file) {
        throw std::runtime_error(
            "Cannot write " + filename
            + ". Please create the results/ folder."
        );
    }

    // CSV 表头
    file << "time,x,y,yaw_rad,target_x,target_y,"
            "steering_deg,error_m\n";

    file << std::fixed << std::setprecision(8);

    // 初始化车辆
    VehicleState state = scenario.initial;

    // 误差统计
    double sq_sum = 0.0;
    double max_error = 0.0;
    double max_steer_deg = 0.0;
    double final_error = 0.0;

    int steps = static_cast<int>(
        DURATION / DT
    );

    // 开始闭环仿真
    for (int i = 0; i < steps; ++i) {

        // 1. 当前横向跟踪误差
        double err =
            lateral_error(state, scenario.circular);

        // 2. 根据前视距离选择目标点
        Point target =
            find_target(state, scenario.path, lookahead);

        // 3. Pure Pursuit 计算转角
        double delta =
            pure_pursuit_steer(state, target);

        double delta_deg =
            delta * 180.0 / PI;

        // 4. 保存当前状态到 CSV
        file << i * DT << ','
             << state.x << ','
             << state.y << ','
             << state.yaw << ','
             << target.x << ','
             << target.y << ','
             << delta_deg << ','
             << err << '\n';

        // 5. 累计误差
        sq_sum += err * err;

        max_error = std::max(
            max_error, err
        );

        max_steer_deg = std::max(
            max_steer_deg,
            std::abs(delta_deg)
        );

        // 6. 使用自行车模型更新车辆状态
        state = step_bicycle(state, delta);
    }

    // 仿真结束后的最终误差
    final_error =
        lateral_error(state, scenario.circular);

    // 计算 RMSE
    double rmse =
        std::sqrt(sq_sum / steps);

    return {
        rmse,
        max_error,
        final_error,
        max_steer_deg
    };
}

// ==============================
// 10. 主函数：9组对比实验
// ==============================

int main() {

    try {

        // 生成参考路径
        Path straight = make_straight_path();
        Path circle = make_circle_path();

        // 三种测试场景
        std::vector<Scenario> scenes = {

            // 场景1：直线，无初始偏移
            {
                "straight",
                {0.0, 0.0, 0.0},
                straight,
                false
            },

            // 场景2：圆形道路，初始偏移2m
            {
                "circle",
                {
                    CIRCLE_RADIUS + 2.0,
                    0.0,
                    PI / 2.0
                },
                circle,
                true
            },

            // 场景3：直线道路，初始偏移2m
            {
                "offset",
                {0.0, 2.0, 0.0},
                straight,
                false
            }
        };

        // 三种前视距离
        std::vector<double> lookaheads = {
            2.5,
            5.0,
            10.0
        };

        std::cout
            << "Day017 Pure Pursuit: fixed model and paths\n"
            << "speed=" << SPEED
            << " m/s, wheelbase=" << WHEELBASE
            << " m, dt=" << DT
            << " s, time=" << DURATION
            << " s\n\n";

        // 输出表格标题
        std::cout << std::left
                  << std::setw(12) << "Scenario"
                  << std::setw(10) << "Ld(m)"
                  << std::setw(13) << "RMSE(m)"
                  << std::setw(13) << "MaxErr(m)"
                  << std::setw(13) << "EndErr(m)"
                  << "MaxSteer(deg)\n";

        std::cout
            << std::fixed << std::setprecision(4);

        // 运行 3 × 3 = 9 组实验
        for (const Scenario& scene : scenes) {

            for (double ld : lookaheads) {

                Metrics m =
                    run_simulation(scene, ld);

                // 输出实验结果
                std::cout << std::left
                          << std::setw(12) << scene.name
                          << std::setw(10) << ld
                          << std::setw(13) << m.rmse
                          << std::setw(13) << m.max_error
                          << std::setw(13) << m.final_error
                          << m.max_steer_deg
                          << '\n';
            }
        }

        std::cout
            << "\nCSV files saved under results/\n";

    } catch (const std::exception& error) {

        std::cerr
            << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
