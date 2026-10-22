#pragma once

#include <cstddef>
#include <vector>
#include "vehicle.hpp"

class Trajectory {
public:
    Trajectory();
    ~Trajectory();

    void addPoint(const VehicleState& state);
    void print() const;

    // 查询轨迹点数量
    std::size_t size() const;

    // 按索引读取轨迹点
    const VehicleState& at(std::size_t index) const;

private:
    std::vector<VehicleState> points_;
};