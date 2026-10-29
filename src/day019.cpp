
#include <iostream>
#include <iomanip>
#include <algorithm>

// 车辆状态
struct VehicleState {
    double speed = 0.0;
};

// P 比例控制器
class PController {
public:
    explicit PController(double kp) : kp_(kp) {}

    double compute(double target, double current) const {
        double error = target - current;
        return kp_ * error;
    }

private:
    double kp_;
};

int main() {
    const double dt = 0.1;       // 仿真时间步长，单位 s
    const double kp = 0.5;       // 比例增益
    const double max_acc = 2.0;  // 最大加速度
    const double min_acc = -3.0; // 最大减速度
    const double drag = 0.1;     // 简化速度阻力系数

    VehicleState car;
    PController controller(kp);

    std::cout << "time,target_speed,actual_speed,control\n";
    std::cout << std::fixed << std::setprecision(3);

    for (int i = 0; i <= 120; ++i) {
        double t = i * dt;

        // 1秒时目标速度由0跳变为10 m/s
        double target = (t < 1.0) ? 0.0 : 10.0;

        // 计算比例控制量
        double acc_cmd = controller.compute(target, car.speed);

        // 加速度限幅
        acc_cmd = std::clamp(acc_cmd, min_acc, max_acc);

        // 记录更新前的车辆状态
        std::cout << t << ","
                  << target << ","
                  << car.speed << ","
                  << acc_cmd << "\n";

        // 简化车辆纵向动力学
        double actual_acc = acc_cmd - drag * car.speed;

        // Euler积分更新车辆速度
        car.speed += actual_acc * dt;
        car.speed = std::max(0.0, car.speed);
    }

    return 0;
}
