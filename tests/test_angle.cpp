#include <gtest/gtest.h>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

// 将角度归一化到 [-PI, PI)
double normalizeAngle(double angle) {
    double result = std::fmod(angle + PI, 2.0 * PI);

    if (result < 0.0) {
        result += 2.0 * PI;
    }

    return result - PI;
}

// 测试 0 弧度
TEST(AngleTest, ZeroAngle) {
    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-9);
}

// 测试完整旋转一圈
TEST(AngleTest, FullRotation) {
    EXPECT_NEAR(normalizeAngle(2.0 * PI), 0.0, 1e-9);
}

// 测试 350 度归一化为 -10 度
TEST(AngleTest, LargeAngle) {
    double input = 350.0 * PI / 180.0;
    double expected = -10.0 * PI / 180.0;

    EXPECT_NEAR(normalizeAngle(input), expected, 1e-9);
}
