
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

struct VehicleState {
    double x;
    double y;
    double yaw;
    double speed;
};

// Day 009 学过的 Euler 状态更新
void step(VehicleState* state,
          double steering,
          double acceleration,
          double dt,
          double wheelbase) {
    state->x += state->speed * std::cos(state->yaw) * dt;
    state->y += state->speed * std::sin(state->yaw) * dt;
    state->yaw += state->speed / wheelbase
                  * std::tan(steering) * dt;
    state->speed += acceleration * dt;
}

// 根据场景返回前轮转角
double getSteering(const std::string& type, double t) {
    if (type == "straight") {
        return 0.0;
    }

    if (type == "circle") {
        return std::atan(2.8 / 10.0);
    }

    if (type == "s_curve") {
        if (t < 4.0) return 0.16;
        if (t < 12.0) return -0.16;
        return 0.16;
    }

    throw std::invalid_argument("Unknown trajectory type");
}

// 生成轨迹并保存为 CSV
void generateTrajectory(const std::string& type) {
    constexpr double dt = 0.1;
    constexpr double wheelbase = 2.8;
    constexpr int steps = 160;

    VehicleState car{0.0, 0.0, 0.0, 4.0};

    std::string filename = "results/day011_" + type + ".csv";
    std::ofstream file(filename);

    if (!file) {
        throw std::runtime_error("Cannot open " + filename);
    }

    file << "time_s,x_m,y_m,yaw_rad,speed_mps,steering_rad\n";
    file << std::fixed << std::setprecision(6);

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double steering = getSteering(type, t);

        file << t << ","
             << car.x << ","
             << car.y << ","
             << car.yaw << ","
             << car.speed << ","
             << steering << "\n";

        if (i < steps) {
            step(&car, steering, 0.0, dt, wheelbase);
        }
    }

    std::cout << "Generated: " << filename << "\n";
}

int main() {
    std::filesystem::create_directories("results");

    generateTrajectory("straight");
    generateTrajectory("circle");
    generateTrajectory("s_curve");

    return 0;
}
