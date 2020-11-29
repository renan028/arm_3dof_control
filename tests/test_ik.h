#pragma once

#include <gtest/gtest.h> 
#include <robot.h>

using namespace remy_robot_control;

const std::vector<std::vector<float>> joints = {
    {1, 0.75, 2.5}, 
    {-2, 0.5, 3},
    {1.25, 0.7, 2.0},
    {-1, 1.5, 3.1},
    {-3.1, 0.6, 1.5},
    {3.14, 1.57, 3.14}, // singularity
    {-3.14, 1.57, 3.14}, // singularity
    {-3.14, -1.57, 3.14}, // singularity
    {-3.14, 1.57, 3.14} // singularity
};

const std::vector<std::vector<float>> xyz = {
    {4.694, 7.311, 2.867},
    {-4.039, -8.825, 0.6432},
    {2.934, 8.829, 5.358},
    {5.291, -8.24, 0.019},
    {-11.59, -0.482, 7.139},
    {-9.992, 0.01591, 0},
    {-9.992, -0.01591, 0},
    {-10.01, -0.01594, 0},
    {-9.992, -0.01591, 0}
};

TEST(IKTest, analytical) 
{
  Robot robot;  
  for (size_t i = 0; i < xyz.size(); ++i) {
    auto result = robot.inverseKinematics(xyz[i][0], xyz[i][1], xyz[i][2]);
    auto xyz_result = robot.forwardKinematics(result);
    EXPECT_NEAR(xyz_result[0], xyz[i][0], 1e-2);
    EXPECT_NEAR(xyz_result[1], xyz[i][1], 1e-2);
    EXPECT_NEAR(xyz_result[2], xyz[i][2], 1e-2);
  }
}

TEST(IKTest, transpose) 
{
  Robot robot;  
  robot.setIk(IkType::transpose);
  for (size_t i = 0; i < 5; ++i) {
    auto result = robot.inverseKinematics(xyz[i][0], xyz[i][1], xyz[i][2]);
    auto xyz_result = robot.forwardKinematics(result);
    EXPECT_NEAR(xyz_result[0], xyz[i][0], 1e-2);
    EXPECT_NEAR(xyz_result[1], xyz[i][1], 1e-2);
    EXPECT_NEAR(xyz_result[2], xyz[i][2], 1e-2);
  }
}

TEST(IKTest, damped) 
{
  Robot robot;  
  robot.setIk(IkType::damped);
  for (size_t i = 0; i < 5; ++i) {
    auto result = robot.inverseKinematics(xyz[i][0], xyz[i][1], xyz[i][2]);
    auto xyz_result = robot.forwardKinematics(result);
    EXPECT_NEAR(xyz_result[0], xyz[i][0], 1e-2);
    EXPECT_NEAR(xyz_result[1], xyz[i][1], 1e-2);
    EXPECT_NEAR(xyz_result[2], xyz[i][2], 1e-2);
  }
}
