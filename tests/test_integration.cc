#include <gtest/gtest.h> 
#include <control.h>
#include <robot_system.h>
#include <fstream>
#include <settings.h>

using namespace remy_robot_control;

TEST(Integration, system) 
{
  std::string input = std::string(TEST_DIR) + std::string("/input.in");
  Control control(std::move(input));
  RobotSystem system;

  system.start(control.connection);
  control.start(system.connection);
  std::this_thread::sleep_for(std::chrono::milliseconds(11000));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}