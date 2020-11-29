#include <gtest/gtest.h> 
#include <json.hpp>
#include <utils.h>
#include <fstream>
#include <settings.h>

using namespace remy_robot_control;
using json = nlohmann::json;

TEST(JsonParser, ControlSettings) 
{
  std::string input = std::string(TEST_DIR) + std::string("/config_test.json");
  std::ifstream i(input);
  json j;
  i >> j;
  auto control_settings = parseControlSetting(j);
  ASSERT_EQ(control_settings.control_type, ControlType::analytical);
  ASSERT_EQ(control_settings.frequency, 14);
}

TEST(JsonParser, RobotSettings) 
{
  std::string input = std::string(TEST_DIR) + std::string("/config_test.json");
  std::ifstream i(input);
  json j;
  i >> j;
  auto robot_settings = parseRobotSetting(j);
  ASSERT_EQ(robot_settings.fktype, FkType::generic);
  ASSERT_EQ(robot_settings.iktype, IkType::damped);
  ASSERT_FLOAT_EQ(robot_settings.joints_min[0], -1);
  ASSERT_FLOAT_EQ(robot_settings.joints_min[1], -0.5);
  ASSERT_FLOAT_EQ(robot_settings.joints_min[2], -2);
  ASSERT_FLOAT_EQ(robot_settings.joints_max[0], 0.5);
  ASSERT_FLOAT_EQ(robot_settings.joints_max[1], 0.1);
  ASSERT_FLOAT_EQ(robot_settings.joints_max[2], 1);
}

TEST(JsonParser, SystemSettings) 
{
  std::string input = std::string(TEST_DIR) + std::string("/config_test.json");
  std::ifstream i(input);
  json j;
  i >> j;
  auto system_settings = parseSystemSetting(j);
  ASSERT_EQ(system_settings.frequency, 1);
  ASSERT_EQ(system_settings.save_output, false);
  ASSERT_EQ(system_settings.encoder_resolution, 1000);
}
