
// remy
#include <robot_system.h>
#include <control.h>
#include <utils.h>

// std
#include <fstream>

// 3rdparty
#include <json.hpp>

using json = nlohmann::json;

int main(int argc, char **argv) {
  assert(argc == 3);
  
  remy_robot_control::Control control(argv[1]);
  remy_robot_control::RobotSystem system;

  std::ifstream i(argv[2]);
  json j;
  i >> j;
  
  control.setSettings(remy_robot_control::parseControlSetting(j));
  control.setRobotSettings(remy_robot_control::parseRobotSetting(j));
  
  system.setSettings(remy_robot_control::parseSystemSetting(j));
  system.setRobotSettings(remy_robot_control::parseRobotSetting(j));

  system.start(control.connection);
  control.start(system.connection);
  std::this_thread::sleep_for(std::chrono::milliseconds(11000));
}