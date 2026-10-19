#include <iostream>
#include <vector>
#include <array>
#include <optional>

// 定义一个轨迹点
struct TrajectoryPoint {
    double x;
    double y;
    double speed;
};

// 安全地获取轨迹点
std::optional<TrajectoryPoint> getPoint(
    const std::vector<TrajectoryPoint>& trajectory,
    std::size_t index
) {
    if (index >= trajectory.size()) {
        return std::nullopt;
    }

    return trajectory.at(index);
}

int main() {

    // 1. array：保存初始位置
    std::array<double, 2> start = {0.0, 0.0};

    // 2. vector：保存整条轨迹
    std::vector<TrajectoryPoint> trajectory;

    // 3. 生成 10 个轨迹点
    for (int i = 0; i < 10; i++) {
        TrajectoryPoint point;

        point.x = start[0] + i * 1.0;
        point.y = start[1] + i * 0.5;
        point.speed = 2.0;

        trajectory.push_back(point);
    }

    // 4. 打印轨迹
    std::cout << "轨迹点数量: "
              << trajectory.size() << "\n";

    for (const auto& point : trajectory) {
        std::cout << "x=" << point.x
                  << ", y=" << point.y
                  << ", speed=" << point.speed
                  << "\n";
    }

    // 5. optional：检查不存在的轨迹点
    auto result = getPoint(trajectory, 20);

    if (result.has_value()) {
        std::cout << "找到轨迹点\n";
    } else {
        std::cout << "轨迹点不存在\n";
    }

    // 6. 检查空轨迹
    std::vector<TrajectoryPoint> emptyTrajectory;

    if (emptyTrajectory.empty()) {
        std::cout << "空轨迹检测成功\n";
    }

    return 0;
}