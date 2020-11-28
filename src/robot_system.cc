#include <robot_system.h>
#include <utils.h>
#include <iostream>

namespace remy_robot_control {

RobotSystem::RobotSystem() :
  robot(std::make_unique<Robot>(0,0,0)),
  connection(std::make_shared<Connection>()),
  stop_(false),
  control_signal(Eigen::Vector3f::Zero()),
  clock(std::chrono::system_clock::now()) 
{
}

RobotSystem::~RobotSystem(){
  stop();
};

void RobotSystem::start(std::weak_ptr<Connection> con) {
  stop();
  stop_ = false;
  thread = std::thread(&RobotSystem::main, this, con);  
}

void RobotSystem::stop() {
  stop_ = true;
  if (thread.joinable()) thread.join();
}

void RobotSystem::main(std::weak_ptr<Connection> con) {
  connection->open();
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    else break;
  }

  clock = std::chrono::system_clock::now();
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) break;
    if (stop_) {
      break;
    }
    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock;
    robot->update(control_signal, diff.count());
    auto q = robot->getJoints();
    mockEncoderPrecisionLost(q);
    auto qc = eigen3fToUchar3(q);
    connection->send(qc);
    
    std::vector<unsigned char> control;
    conn->receive(control);
    control_signal = uchar3ToEigen3f(control);
    clock = new_clock;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  connection->close();
}
} // end namespace remy_robot_control