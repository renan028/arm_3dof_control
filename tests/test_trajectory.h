#pragma once

#include <gtest/gtest.h> 
#include <types.h>

using namespace remy_robot_control;

TEST(Trajectory, update) 
{
  const std::vector<Eigen::Vector4f> waypoints = {
    Eigen::Vector4f(20.0, 0.0, 0.0, 0.0),
    Eigen::Vector4f(17.0, 0.0, 0.0, 1.5),
    Eigen::Vector4f(15.0, 1.5, 1.5, 3.5),
    Eigen::Vector4f(15.0, -1.5, 1.5, 5.0),
    Eigen::Vector4f(15.0, -1.5, -1.5, 7.0),
    Eigen::Vector4f(15.0, 1.5, -1.5, 9.0),
    Eigen::Vector4f(20.0, 0.0, 0.0, 10.0)
  };

  Trajectory traj(waypoints);
  traj.update(0);
  ASSERT_FLOAT_EQ(traj.x[0], 20);
  ASSERT_FLOAT_EQ(traj.x[1], 0);
  ASSERT_FLOAT_EQ(traj.x[2], 0);
  ASSERT_FLOAT_EQ(traj.v[0], 0);
  ASSERT_FLOAT_EQ(traj.v[1], 0);
  ASSERT_FLOAT_EQ(traj.v[2], 0);

  traj.update(1);
  ASSERT_FLOAT_EQ(traj.x[0], 18);
  ASSERT_FLOAT_EQ(traj.x[1], 0);
  ASSERT_FLOAT_EQ(traj.x[2], 0);
  ASSERT_FLOAT_EQ(traj.v[0], -2);
  ASSERT_FLOAT_EQ(traj.v[1], 0);
  ASSERT_FLOAT_EQ(traj.v[2], 0);

  traj.update(1.5);
  ASSERT_FLOAT_EQ(traj.x[0], 17);
  ASSERT_FLOAT_EQ(traj.x[1], 0);
  ASSERT_FLOAT_EQ(traj.x[2], 0);
  ASSERT_FLOAT_EQ(traj.v[0], -2);
  ASSERT_FLOAT_EQ(traj.v[1], 0);
  ASSERT_FLOAT_EQ(traj.v[2], 0);

  traj.update(2);
  ASSERT_FLOAT_EQ(traj.x[0], 16.5);
  ASSERT_FLOAT_EQ(traj.x[1], 0.375);
  ASSERT_FLOAT_EQ(traj.x[2], 0.375);
  ASSERT_FLOAT_EQ(traj.v[0], -1);
  ASSERT_FLOAT_EQ(traj.v[1], 0.75);
  ASSERT_FLOAT_EQ(traj.v[2], 0.75);

  traj.update(3.5);
  ASSERT_FLOAT_EQ(traj.x[0], 15);
  ASSERT_FLOAT_EQ(traj.x[1], 1.5);
  ASSERT_FLOAT_EQ(traj.x[2], 1.5);
  ASSERT_FLOAT_EQ(traj.v[0], -1);
  ASSERT_FLOAT_EQ(traj.v[1], 0.75);
  ASSERT_FLOAT_EQ(traj.v[2], 0.75);

  traj.update(8);
  ASSERT_FLOAT_EQ(traj.x[0], 15);
  ASSERT_FLOAT_EQ(traj.x[1], 0);
  ASSERT_FLOAT_EQ(traj.x[2], -1.5);
  ASSERT_FLOAT_EQ(traj.v[0], 0);
  ASSERT_FLOAT_EQ(traj.v[1], 1.5);
  ASSERT_FLOAT_EQ(traj.v[2], 0);


  traj.update(10);
  ASSERT_FLOAT_EQ(traj.x[0], 20);
  ASSERT_FLOAT_EQ(traj.x[1], 0);
  ASSERT_FLOAT_EQ(traj.x[2], 0);
  ASSERT_FLOAT_EQ(traj.v[0], 5);
  ASSERT_FLOAT_EQ(traj.v[1], -1.5);
  ASSERT_FLOAT_EQ(traj.v[2], 1.5);
}