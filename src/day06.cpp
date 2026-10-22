#include <iostream>
#include <memory>
#include <utility>

#include "vehicle.hpp"
#include "trajectory.hpp"

int main() {
    VehicleState car{0.0, 0.0, 5.0};
    ControlCommand cmd{0.0, 0.0};

    std::cout << "车辆初始速度: "
              << car.speed << "\n";

    std::cout << "加速度指令: "
              << cmd.acceleration << "\n";

    {
        auto trajectory = std::make_unique<Trajectory>();

        trajectory->addPoint({0.0, 0.0, 5.0});
        trajectory->addPoint({1.0, 0.0, 5.0});
        trajectory->addPoint({2.0, 0.0, 5.0});

        auto new_owner = std::move(trajectory);

        std::cout << "所有权已转移\n";

        if (trajectory == nullptr) {
            std::cout << "旧指针已经为空\n";
        }

        new_owner->print();
    }

    std::cout << "程序结束\n";
    return 0;
}
