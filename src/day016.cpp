
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <algorithm>

constexpr double PI = 3.14159265358979323846;

// ==========================================
// 1. 定义数据结构
// ==========================================

// 二维路径点
struct Point {
    double x;
    double y;
};

// 车辆状态
struct VehicleState {
    double x;
    double y;
    double yaw;
};

// ==========================================
// 2. 生成圆形参考路径
// ==========================================
std::vector<Point> generateCirclePath(
    double radius, int num_points)
{
    std::vector<Point> path;

    for (int i = 0; i < num_points; ++i) {
        double theta = 2.0 * PI * i / num_points;

        Point p;
        p.x = radius * std::cos(theta);
        p.y = radius * std::sin(theta);

        path.push_back(p);
    }

    return path;
}

// ==========================================
// 3. 计算两个点之间的距离
// ==========================================
double distance(Point a, Point b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;

    return std::sqrt(dx * dx + dy * dy);
}

// ==========================================
// 4. 搜索前视目标点
// ==========================================
int findTargetIndex(
    const std::vector<Point>& path,
    Point vehicle,
    double lookahead)
{
    if (path.empty() || lookahead <= 0.0) {
        return -1;
    }

    int n = static_cast<int>(path.size());

    // 寻找距离车辆最近的路径点
    int nearest_index = 0;
    double min_distance = distance(vehicle, path[0]);

    for (int i = 1; i < n; ++i) {
        double d = distance(vehicle, path[i]);

        if (d < min_distance) {
            min_distance = d;
            nearest_index = i;
        }
    }

    // 沿闭合路径向前寻找目标点
    for (int step = 0; step < n; ++step) {
        int index = (nearest_index + step) % n;

        double d = distance(vehicle, path[index]);

        if (d >= lookahead) {
            return index;
        }
    }

    return -1;
}

// ==========================================
// 5. Pure Pursuit 控制器
// ==========================================
double purePursuitControl(
    Point vehicle,
    double yaw,
    Point target,
    double wheelbase)
{
    double dx = target.x - vehicle.x;
    double dy = target.y - vehicle.y;

    // 目标方向角
    double target_angle = std::atan2(dy, dx);

    // 航向偏角
    double alpha = target_angle - yaw;

    // 角度归一化
    alpha = std::atan2(
        std::sin(alpha),
        std::cos(alpha)
    );

    // 实际前视距离
    double Ld = distance(vehicle, target);

    if (Ld < 1e-6) {
        return 0.0;
    }

    // 期望曲率
    double curvature = 2.0 * std::sin(alpha) / Ld;

    // 前轮转角
    double delta = std::atan(wheelbase * curvature);

    return delta;
}

// ==========================================
// 6. 自行车运动模型
// ==========================================
void updateVehicle(
    VehicleState& state,
    double speed,
    double delta,
    double wheelbase,
    double dt)
{
    // 更新车辆位置
    state.x += speed * std::cos(state.yaw) * dt;
    state.y += speed * std::sin(state.yaw) * dt;

    // 更新车辆航向角
    state.yaw +=
        speed / wheelbase * std::tan(delta) * dt;

    // 航向角归一化
    state.yaw = std::atan2(
        std::sin(state.yaw),
        std::cos(state.yaw)
    );
}

// ==========================================
// 7. 主函数：闭环仿真
// ==========================================
int main()
{
    // ---------- 参考路径参数 ----------
    double radius = 20.0;
    int num_points = 100;

    std::vector<Point> path =
        generateCirclePath(radius, num_points);

    // ---------- 车辆参数 ----------
    double wheelbase = 2.8;
    double speed = 5.0;
    double lookahead = 5.0;

    // 最大前轮转角：30度
    double max_steer = 30.0 * PI / 180.0;

    // ---------- 仿真参数 ----------
    double dt = 0.05;
    double simulation_time = 30.0;

    int steps = static_cast<int>(simulation_time / dt);

    // 车辆初始状态
    // 比参考圆半径大2米，模拟初始偏离
    VehicleState vehicle{22.0, 0.0, PI / 2.0};

    // ---------- 保存轨迹 ----------
    std::ofstream csv("results/day016_trajectory.csv");

    if (!csv.is_open()) {
        std::cerr << "Cannot open trajectory CSV file.\n";
        return 1;
    }

    csv << "time,x,y,yaw_rad,target_x,target_y,"
           "steering_deg,radial_error\n";

    csv << std::fixed << std::setprecision(6);

    // ---------- 误差统计 ----------
    double sum_squared_error = 0.0;
    double max_error = 0.0;

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "Pure Pursuit Simulation\n";
    std::cout << "Reference radius: " << radius << " m\n";
    std::cout << "Path points: " << path.size() << "\n";
    std::cout << "Initial position: ("
              << vehicle.x << ", " << vehicle.y << ")\n";
    std::cout << "Speed: " << speed << " m/s\n";
    std::cout << "Lookahead: " << lookahead << " m\n\n";

    std::cout << "Time(s)\tX(m)\tY(m)\tError(m)\tSteer(deg)\n";

    // ======================================
    // 核心：闭环控制循环
    // ======================================
    for (int step = 0; step < steps; ++step) {

        double time = step * dt;

        // 当前车辆位置
        Point current{vehicle.x, vehicle.y};

        // 1. 寻找前视目标点
        int target_index =
            findTargetIndex(path, current, lookahead);

        if (target_index < 0) {
            std::cerr << "Target not found.\n";
            return 1;
        }

        Point target = path[target_index];

        // 2. Pure Pursuit 计算前轮转角
        double delta = purePursuitControl(
            current,
            vehicle.yaw,
            target,
            wheelbase
        );

        // 3. 限制最大前轮转角
        delta = std::clamp(
            delta,
            -max_steer,
            max_steer
        );

        // 4. 计算圆形路径径向误差
        double radial_distance =
            std::hypot(vehicle.x, vehicle.y);

        double error = radial_distance - radius;

        // 5. 累计误差
        sum_squared_error += error * error;

        max_error = std::max(
            max_error,
            std::abs(error)
        );

        // 6. 保存当前车辆状态
        csv << time << ","
            << vehicle.x << ","
            << vehicle.y << ","
            << vehicle.yaw << ","
            << target.x << ","
            << target.y << ","
            << delta * 180.0 / PI << ","
            << error << "\n";

        // 7. 每隔1秒输出一次
        if (step % 20 == 0) {
            std::cout
                << time << "\t"
                << vehicle.x << "\t"
                << vehicle.y << "\t"
                << error << "\t"
                << delta * 180.0 / PI
                << "\n";
        }

        // 8. 自行车模型更新车辆状态
        updateVehicle(
            vehicle,
            speed,
            delta,
            wheelbase,
            dt
        );
    }

    csv.close();

    // ---------- 最终误差统计 ----------
    double rmse = std::sqrt(
        sum_squared_error / steps
    );

    double final_error =
        std::hypot(vehicle.x, vehicle.y) - radius;

    std::cout << "\nSimulation finished.\n";

    std::cout << "RMS radial error: "
              << rmse << " m\n";

    std::cout << "Max absolute radial error: "
              << max_error << " m\n";

    std::cout << "Final radial error: "
              << final_error << " m\n";

    std::cout
        << "Trajectory saved to results/day016_trajectory.csv\n";

    return 0;
}
