
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

constexpr double PI = 3.14159265358979323846;
constexpr double L = 2.7;
constexpr double DT = 0.05;

// 路径点
struct Point {
    double x, y;
};

// 车辆状态（位置为后轴中心）
struct VehicleState {
    double x, y, yaw, v;
};

// 轨迹跟踪误差
struct TrackingError {
    double heading;
    double lateral;
    Point nearest;
};

// 角度归一化
double normalizeAngle(double a) {
    while (a > PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

// 生成 S 形参考路径
std::vector<Point> createReferencePath() {
    std::vector<Point> path;

    for (int i = 0; i <= 500; ++i) {
        double x = i * 0.2;
        double y = 2.0 * std::sin(0.12 * x);
        path.push_back({x, y});
    }

    return path;
}

// 计算前轴相对参考路径的误差
TrackingError calculateError(
    const VehicleState& car,
    const std::vector<Point>& path
) {
    Point front{
        car.x + L * std::cos(car.yaw),
        car.y + L * std::sin(car.yaw)
    };

    double bestDist2 =
        std::numeric_limits<double>::infinity();

    TrackingError best{};

    for (std::size_t i = 0;
         i + 1 < path.size(); ++i) {

        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        double len2 = dx*dx + dy*dy;

        if (len2 <= 0.0) continue;

        // 将前轴中心投影到参考路径线段
        double t = std::clamp(
            ((front.x - path[i].x)*dx +
             (front.y - path[i].y)*dy) / len2,
            0.0, 1.0
        );

        Point proj{
            path[i].x + t*dx,
            path[i].y + t*dy
        };

        double ex = proj.x - front.x;
        double ey = proj.y - front.y;
        double dist2 = ex*ex + ey*ey;

        if (dist2 < bestDist2) {
            bestDist2 = dist2;

            double refYaw = std::atan2(dy, dx);

            best.heading =
                normalizeAngle(refYaw - car.yaw);

            // 路径在前轴左侧时为正
            best.lateral =
                (dx*ey - dy*ex) / std::sqrt(len2);

            best.nearest = proj;
        }
    }

    return best;
}

// Stanley 控制器
double stanleyControl(
    const TrackingError& e,
    double v,
    double k
) {
    const double epsilon = 0.1;
    const double maxSteer = 0.6;

    double delta =
        e.heading +
        std::atan2(k * e.lateral,
                   std::max(0.0, v) + epsilon);

    return std::clamp(
        delta, -maxSteer, maxSteer
    );
}

// 自行车模型：更新一步状态
void step(VehicleState& car, double delta) {
    double oldYaw = car.yaw;

    car.x += car.v * std::cos(oldYaw) * DT;
    car.y += car.v * std::sin(oldYaw) * DT;

    car.yaw = normalizeAngle(
        oldYaw +
        car.v / L * std::tan(delta) * DT
    );
}

int main() {
    auto path = createReferencePath();

    // 初始车辆状态
    VehicleState car{0.0, -1.0, 0.0, 5.0};

    const double k = 1.0;
    const int steps = 300;

    std::ofstream csv(
        "results/day018_trajectory.csv"
    );

    if (!csv) {
        std::cerr << "无法打开结果文件\n";
        return 1;
    }

    csv << "time,x,y,yaw,steer,"
           "cross_track_error,heading_error,"
           "ref_x,ref_y\n";

    csv << std::fixed << std::setprecision(6);

    double sumSq = 0.0;
    double maxAbs = 0.0;

    for (int i = 0; i < steps; ++i) {
        double time = i * DT;

        // 1. 计算跟踪误差
        TrackingError e =
            calculateError(car, path);

        // 2. Stanley 转向控制
        double delta =
            stanleyControl(e, car.v, k);

        // 3. 保存结果
        csv << time << ","
            << car.x << ","
            << car.y << ","
            << car.yaw << ","
            << delta << ","
            << e.lateral << ","
            << e.heading << ","
            << e.nearest.x << ","
            << e.nearest.y << "\n";

        sumSq += e.lateral * e.lateral;
        maxAbs = std::max(
            maxAbs, std::abs(e.lateral)
        );

        // 4. 更新车辆状态
        step(car, delta);
    }

    std::cout << "Simulation time: "
              << steps * DT << " s\n";

    std::cout << "Lateral error RMSE: "
              << std::sqrt(sumSq / steps)
              << " m\n";

    std::cout << "Maximum lateral error: "
              << maxAbs << " m\n";

    std::cout << "Final position: ("
              << car.x << ", "
              << car.y << ") m\n";

    std::cout
        << "Saved: results/day018_trajectory.csv\n";

    return 0;
}
