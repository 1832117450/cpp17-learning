#include <gtest/gtest.h>
#include <stdexcept>

#include "trajectory.hpp"

// 测试1：刚创建的轨迹应当为空
TEST(TrajectoryTest, InitiallyEmpty) {
    Trajectory trajectory;

    EXPECT_EQ(trajectory.size(), 0U);
}

// 测试2：添加轨迹点后能够正确读取
TEST(TrajectoryTest, AddAndReadPoints) {
    Trajectory trajectory;

    trajectory.addPoint({0.0, 0.0, 5.0});
    trajectory.addPoint({1.0, 2.0, 6.0});

    EXPECT_EQ(trajectory.size(), 2U);
    EXPECT_DOUBLE_EQ(trajectory.at(1).x, 1.0);
    EXPECT_DOUBLE_EQ(trajectory.at(1).y, 2.0);
    EXPECT_DOUBLE_EQ(trajectory.at(1).speed, 6.0);
}

// 测试3：访问不存在的轨迹点应当抛出异常
TEST(TrajectoryTest, InvalidIndex) {
    Trajectory trajectory;

    EXPECT_THROW(trajectory.at(0), std::out_of_range);
}
