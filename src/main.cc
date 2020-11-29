#include <robot_system.h>
#include <control.h>

int main(int argc, char **argv) {
  assert(argc == 2);
  
  remy_robot_control::Control control(argv[1]);
  remy_robot_control::RobotSystem system;

  system.start(control.connection);
  control.start(system.connection);
  std::this_thread::sleep_for(std::chrono::milliseconds(11000));
}