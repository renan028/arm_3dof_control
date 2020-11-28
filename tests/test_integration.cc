#include <gtest/gtest.h> 
#include <control.h>
#include <robot_system.h>
#include <fstream>
#include <settings.h>

using namespace remy_robot_control;

void assertPosition(const Eigen::Vector3f& p, float t) {
  float tol = 1e-2;
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

TEST(Integration, system) 
{
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  RobotSystem system;

  control.setControlStrategy(Control::ControlType::analytical);

  system.start(control.connection);
  control.start(system.connection);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}