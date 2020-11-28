#pragma once

#include <gtest/gtest.h> 
#include <robot.h>

using namespace remy_robot_control;

const std::vector<std::vector<double>> joints_set = {
    {1, 0.75, 2.5}, 
    {-2, 0.5, 3},
    {1.25, 0.7, 2.0},
    {-1, 1.5, 3.1},
    {-3.1, 0.6, 1.5},
    {3.14, 1.57, 3.14},
    {-3.14, 1.57, 3.14},
    {-3.14, -1.57, 3.14},
    {-3.14, -1.57, -3.14}
};

const std::vector<std::vector<double>> solution_set = {
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

TEST(FKTest, genericFK) 
{
  Robot robot;  
  for (size_t i = 0; i < joints_set.size(); ++i) {
    auto result = robot.forwardKinematics({joints_set[i][0], joints_set[i][1], 
      joints_set[i][2]});
    EXPECT_NEAR(result[0], solution_set[i][0], 1e-2);
    EXPECT_NEAR(result[1], solution_set[i][1], 1e-2);
    EXPECT_NEAR(result[2], solution_set[i][2], 1e-2);
  }
}

TEST(FKTest, fastFK) 
{
  Robot robot;  
  robot.setFk(Robot::FkType::fast);
  for (size_t i = 0; i < joints_set.size(); ++i) {
    auto result = robot.forwardKinematics({joints_set[i][0], joints_set[i][1], 
      joints_set[i][2]});
    EXPECT_NEAR(result[0], solution_set[i][0], 1e-2);
    EXPECT_NEAR(result[1], solution_set[i][1], 1e-2);
    EXPECT_NEAR(result[2], solution_set[i][2], 1e-2);
  }
}