#include <gtest/gtest.h> 
#include <control.h>
#include <fstream>
#include <settings.h>

using namespace remy_robot_control;

TEST(Control, init) 
{
  const std::vector<Eigen::Vector4d> waypoints = {
    Eigen::Vector4d(20.0, 0.0, 0.0, 0.0),
    Eigen::Vector4d(17.0, 0.0, 0.0, 1.5),
    Eigen::Vector4d(15.0, 1.5, 1.5, 3.5),
    Eigen::Vector4d(15.0, -1.5, 1.5, 5.0),
    Eigen::Vector4d(15.0, -1.5, -1.5, 7.0),
    Eigen::Vector4d(15.0, 1.5, -1.5, 9.0),
    Eigen::Vector4d(20.0, 0.0, 0.0, 10.0)
  };

  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  auto waypoints_ = control.getWaypoints();

  ASSERT_EQ(waypoints.size(), waypoints_.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    ASSERT_DOUBLE_EQ(waypoints[i][0], waypoints_[i][0]);
    ASSERT_DOUBLE_EQ(waypoints[i][1], waypoints_[i][1]);
    ASSERT_DOUBLE_EQ(waypoints[i][2], waypoints_[i][2]);
    ASSERT_DOUBLE_EQ(waypoints[i][3], waypoints_[i][3]);
  }
}

void assertPosition(const Eigen::Vector3d& p, double t) {
  double tol = 1e-2;
  if (t == 1.5) {
    EXPECT_NEAR(p[0], 17, tol);
    EXPECT_NEAR(p[1], 0, tol);
    EXPECT_NEAR(p[2], 0, tol);
    return;
  }

  if (t == 3.5) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (t == 3.5) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (t == 5.0) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], -1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (t == 7.0) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], -1.5, tol);
    EXPECT_NEAR(p[2], -1.5, tol);
    return;
  }

  if (t == 9.0) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], -1.5, tol);
    return;
  }

  if (t == 9.0) {
    EXPECT_NEAR(p[0], 20, tol);
    EXPECT_NEAR(p[1], 0, tol);
    EXPECT_NEAR(p[2], 0, tol);
    return;
  }
}

TEST(Control, feedforward) 
{
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  Robot robot(0, 0, 0);
  double dt = 0.02;
  for (double t = 0; t < 11; t+= 0.02) {
    auto u = control.computeVelocityControl(robot.getJoints(), t);
    robot.update(u, dt);
    auto p = robot.forwardKinematics(robot.getJoints());
    assertPosition(p, t);
  }
}

TEST(Control, analytical) 
{
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  Robot robot(0, 0, 0);
  control.setControlStrategy(Control::ControlType::analytical);
  double dt = 0.02;
  for (double t = 0; t < 11; t+= 0.02) {
    auto u = control.computeVelocityControl(robot.getJoints(), t);
    robot.update(u, dt);
    auto p = robot.forwardKinematics(robot.getJoints());
    assertPosition(p, t);
  }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}