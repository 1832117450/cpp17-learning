
#include <iostream>
#include <iomanip>
#include <algorithm>

struct VehicleState {
    double speed = 0.0;
};

class PIController {
public:
    PIController(double kp, double ki, double dt)
        : kp_(kp), ki_(ki), dt_(dt) {}

    double compute(double target, double current) {
        double error = target - current;

        // 1. 计算候选积分值
        double candidate_integral = integral_ + error * dt_;

        // 2. 计算 PI 控制量
        double raw_control =
            kp_ * error + ki_ * candidate_integral;

        // 3. 加速度限幅
        double control =
            std::clamp(raw_control, -3.0, 2.0);

        // 4. 抗积分饱和：避免在饱和方向继续积分
        bool upper_saturated =
            (raw_control > 2.0 && error > 0.0);

        bool lower_saturated =
            (raw_control < -3.0 && error < 0.0);

        if (!upper_saturated && !lower_saturated) {
            integral_ = candidate_integral;
        }

        return control;
    }

private:
    double kp_;
    double ki_;
    double dt_;
    double integral_ = 0.0;
};

int main() {
    const double dt = 0.1;
    const double kp = 0.5;
    const double ki = 0.2;
    const double drag = 0.1;

    VehicleState car;
    PIController controller(kp, ki, dt);

    std::cout << "time,target_speed,actual_speed,control\n";
    std::cout << std::fixed << std::setprecision(3);

    for (int i = 0; i <= 120; ++i) {
        double t = i * dt;
        double target = (t < 1.0) ? 0.0 : 10.0;

        // PI 控制
        double acc_cmd =
            controller.compute(target, car.speed);

        std::cout << t << ","
                  << target << ","
                  << car.speed << ","
                  << acc_cmd << "\n";

        // 与 P 控制实验保持相同的车辆模型
        double actual_acc = acc_cmd - drag * car.speed;

        car.speed += actual_acc * dt;
        car.speed = std::max(0.0, car.speed);
    }

    return 0;
}
