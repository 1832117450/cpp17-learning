#include "trajectory.hpp"
#include <iostream>

Trajectory::Trajectory() {
    std::cout << "轨迹创建\n";
}

Trajectory::~Trajectory() {
    std::cout << "轨迹销毁\n";
}

void Trajectory::addPoint(const VehicleState& state) {
    points_.push_back(state);
}

void Trajectory::print() const {
    for (const auto& point : points_) {
        std::cout << "x=" << point.x
                  << ", y=" << point.y
                  << ", speed=" << point.speed
                  << "\n";
    }
}

std::size_t Trajectory::size() const {
    return points_.size();
}

const VehicleState& Trajectory::at(std::size_t index) const {
    return points_.at(index);
}
