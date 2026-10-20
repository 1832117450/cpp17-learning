#include <iostream>

class VehicleState {
public:
    VehicleState() {
        std::cout << "1. VehicleState 构造函数\n";
    }

    ~VehicleState() {
        std::cout << "3. VehicleState 析构函数\n";
    }

    void drive() const {
        std::cout << "2. 车辆正在行驶\n";
    }
};

int main() {
    std::cout << "程序开始\n";

    {
        VehicleState car;
        car.drive();
    }

    std::cout << "程序结束\n";

    return 0;
}
