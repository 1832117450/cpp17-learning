#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// 车辆坐标 -> 世界坐标
Eigen::Vector2d vehicleToWorld(
    const Eigen::Vector2d& local,
    const Eigen::Vector2d& position,
    double yaw)
{
    Eigen::Matrix2d R;

    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw),  std::cos(yaw);

    return R * local + position;
}

// 世界坐标 -> 车辆坐标
Eigen::Vector2d worldToVehicle(
    const Eigen::Vector2d& world,
    const Eigen::Vector2d& position,
    double yaw)
{
    Eigen::Matrix2d R;

    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw),  std::cos(yaw);

    return R.transpose() * (world - position);
}

int main()
{
    Eigen::Vector2d position(10.0, 5.0);
    Eigen::Vector2d local(2.0, 1.0);

    double yaw = 0.0;//pi / 2.0

    Eigen::Vector2d world =
        vehicleToWorld(local, position, yaw);

    Eigen::Vector2d recovered =
        worldToVehicle(world, position, yaw);

    std::cout << "Local: "
              << local.transpose() << "\n";

    std::cout << "World: "
              << world.transpose() << "\n";

    std::cout << "Recovered: "
              << recovered.transpose() << "\n";

    std::cout << "Error: "
              << (recovered - local).norm() << "\n";

    return 0;
}