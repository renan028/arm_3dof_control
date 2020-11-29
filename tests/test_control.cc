#include <gtest/gtest.h> 
#include <control.h>
#include <fstream>
#include <settings.h>

using namespace remy_robot_control;

TEST(Control, init) 
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
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  auto waypoints_ = control.getWaypoints();

  ASSERT_EQ(waypoints.size(), waypoints_.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    ASSERT_FLOAT_EQ(waypoints[i][0], waypoints_[i][0]);
    ASSERT_FLOAT_EQ(waypoints[i][1], waypoints_[i][1]);
    ASSERT_FLOAT_EQ(waypoints[i][2], waypoints_[i][2]);
    ASSERT_FLOAT_EQ(waypoints[i][3], waypoints_[i][3]);
  }
}

void assertPosition(const Eigen::Vector3f& p, float t) {
  float tol = 1e-1;
  if (abs(t - 1.5) <= 1e-3) {
    EXPECT_NEAR(p[0], 17, tol);
    EXPECT_NEAR(p[1], 0, tol);
    EXPECT_NEAR(p[2], 0, tol);
    return;
  }

  if (abs(t -3.5) <= 1e-3) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (abs(t - 3.5) <= 1e-3) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (abs(t - 5.0) <= 1e-3) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], -1.5, tol);
    EXPECT_NEAR(p[2], 1.5, tol);
    return;
  }

  if (abs(t - 7.0) <= 1e-3) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], -1.5, tol);
    EXPECT_NEAR(p[2], -1.5, tol);
    return;
  }

  if (abs(t - 9.0) <= 1e-3) {
    EXPECT_NEAR(p[0], 15, tol);
    EXPECT_NEAR(p[1], 1.5, tol);
    EXPECT_NEAR(p[2], -1.5, tol);
    return;
  }

  if (abs(t - 10.0) <= 1e-3) {
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
  float dt = 0.02;
  for (float t = 0; t < 11; t+= 0.02) {
    control.computeVelocityControl(robot.getJoints(), t);
    robot.update(control.getControlSignal(), dt);
    auto p = robot.forwardKinematics(robot.getJoints());
    //std::cout << "p: " << p[0] << ", " << p[1] << ", " << p[2] << ", t: " << t << "\n";
    assertPosition(p, t);
  }
}

TEST(Control, analytical) 
{
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  Robot robot(0, 0, 0);
  control.setControlStrategy(Control::ControlType::analytical);
  float dt = 0.02;
  for (float t = 0; t < 11; t+= 0.02) {
    control.computeVelocityControl(robot.getJoints(), t);
    robot.update(control.getControlSignal(), dt);
    auto p = robot.forwardKinematics(robot.getJoints());
    assertPosition(p, t);
  }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}