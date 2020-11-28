#pragma once

#include <gtest/gtest.h> 
#include <utils.h>

using namespace remy_robot_control;

TEST(Data, convert) 
{
  Eigen::Vector3f u(5.12, 4.53, 6.55);
  auto uc = eigen3fToUchar3(u);
  auto uu = uchar3ToEigen3f(uc);
  ASSERT_DOUBLE_EQ(uu[0], u[0]);
  ASSERT_DOUBLE_EQ(uu[1], u[1]);
  ASSERT_DOUBLE_EQ(uu[2], u[2]);
}

TEST(Data, encoder) 
{
  Eigen::Vector3f u(5.121, 4.532, 6.553);
  mockEncoderPrecisionLost(u);
  EXPECT_NEAR(5.1204, u[0], 1e-4);
  EXPECT_NEAR(4.5313, u[1], 1e-4);
  EXPECT_NEAR(6.5516, u[2], 1e-4);
}