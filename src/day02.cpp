#include <iostream>

struct VehicleState {
    double speed;
    bool braking;
};

// 值传递：修改副本
void changeByValue(VehicleState car) {
    car.speed = 20.0;

    std::cout << "Inside value function: "
              << car.speed << '\n';
}

// 引用传递：修改原对象
void changeByReference(VehicleState& car) {
    car.speed = 30.0;
}

// 指针传递：通过地址修改原对象
void emergencyBrake(VehicleState* car) {
    if (car != nullptr) {
        car->speed = 0.0;
        car->braking = true;
    }
}

// const 引用：只读取，不允许修改
void printState(const VehicleState& car) {
    std::cout << "Speed: " << car.speed
              << ", Braking: " << std::boolalpha
              << car.braking << '\n';
}

int main() {
    VehicleState car{10.0, false};

    std::cout << "Initial: ";
    printState(car);

    changeByValue(car);

    std::cout << "After value: ";
    printState(car);

    changeByReference(car);

    std::cout << "After reference: ";
    printState(car);

    emergencyBrake(&car);

    std::cout << "After pointer: ";
    printState(car);

    return 0;
}