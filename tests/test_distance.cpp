#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>

// 计算二维平面上两点之间的距离
double distance(double x1, double y1,
                double x2, double y2) {

    // 检查输入是否有效
    if (!std::isfinite(x1) || !std::isfinite(y1) ||
        !std::isfinite(x2) || !std::isfinite(y2)) {
        throw std::invalid_argument("Invalid coordinate");
    }

    return std::hypot(x2 - x1, y2 - y1);
}

// 测试正常距离
TEST(DistanceTest, BasicCalculation) {
    EXPECT_DOUBLE_EQ(distance(0, 0, 3, 4), 5.0);
}

// 测试两个点重合
TEST(DistanceTest, SamePoint) {
    EXPECT_DOUBLE_EQ(distance(1, 1, 1, 1), 0.0);
}

// 测试非法输入
TEST(DistanceTest, InvalidInput) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    double inf = std::numeric_limits<double>::infinity();

    EXPECT_THROW(distance(nan, 0, 3, 4),
                 std::invalid_argument);

    EXPECT_THROW(distance(0, inf, 3, 4),
                 std::invalid_argument);
}
