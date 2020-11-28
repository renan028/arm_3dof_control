#include <robot_system.h>
#include <utils.h>

namespace remy_robot_control {

RobotSystem::RobotSystem() :
  robot(std::make_unique<Robot>(0,0,0)),
  stop_(false),
  control_signal(Eigen::Vector3f::Zero()),
  clock(std::chrono::system_clock::now()) 
{
}

RobotSystem::~RobotSystem(){
  stop();
};

void RobotSystem::start() {
  stop();
  stop_ = false;
  thread = std::thread(&RobotSystem::start, this);  
}

void RobotSystem::stop() {
  stop_ = true;
  if (thread.joinable()) thread.join();
  thread.join();
}

void RobotSystem::main() {
  connection.open();
  clock = std::chrono::system_clock::now();
  while(true) {
    if (stop_) {
      break;
    }
    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock;
    robot->update(control_signal, diff.count());
    auto q = eigen3fToUchar3(robot->getJoints());
    connection.send(q);
    
    std::vector<unsigned char> control;
    connection.receive(control);
    control_signal = uchar3ToEigen3f(control);
    clock = new_clock;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  connection.close();
}
} // end namespace remy_robot_control