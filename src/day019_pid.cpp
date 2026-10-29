
#include <iostream>
#include <iomanip>
#include <algorithm>

struct VehicleState {
    double speed = 0.0;
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt) {}

    double compute(double target, double current) {
        double error = target - current;

        // P：比例项
        double p = kp_ * error;

        // I：候选积分项
        double candidate_integral = integral_ + error * dt_;
        double i = ki_ * candidate_integral;

        // D：对实际速度求导，避免目标速度阶跃导致微分冲击
        double d = 0.0;
        if (initialized_) {
            d = -kd_ * (current - previous_speed_) / dt_;
        }

        previous_speed_ = current;
        initialized_ = true;

        // PID控制量
        double raw_control = p + i + d;

        // 加速度限幅
        double control = std::clamp(raw_control, -3.0, 2.0);

        // 简单抗积分饱和
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
    double kd_;
    double dt_;

    double integral_ = 0.0;
    double previous_speed_ = 0.0;
    bool initialized_ = false;
};

int main() {
    const double dt = 0.1;

    const double kp = 0.5;
    const double ki = 0.2;
    const double kd = 0.3;

    const double drag = 0.1;

    VehicleState car;
    PIDController controller(kp, ki, kd, dt);

    std::cout << "time,target_speed,actual_speed,control\n";
    std::cout << std::fixed << std::setprecision(3);

    for (int step = 0; step <= 120; ++step) {
        double t = step * dt;

        double target = (t < 1.0) ? 0.0 : 10.0;

        double acc_cmd = controller.compute(
            target, car.speed
        );

        std::cout << t << ","
                  << target << ","
                  << car.speed << ","
                  << acc_cmd << "\n";

        double actual_acc = acc_cmd - drag * car.speed;

        car.speed += actual_acc * dt;
        car.speed = std::max(0.0, car.speed);
    }

    return 0;
}
