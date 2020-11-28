#pragma once

// remy
#include <robot.h>
#include <connection.h>

// std
#include <thread>
#include <atomic>

namespace remy_robot_control {
  
class RobotSystem {
  std::unique_ptr<Robot> robot;
  std::thread thread;
  std::atomic<bool> stop_;
  Eigen::Vector3f control_signal;
  std::chrono::time_point<std::chrono::system_clock> clock;
  
  public:
    RobotSystem();
    ~RobotSystem();

    /* It creates a thread to start the robotic system. */
    void start(std::weak_ptr<Connection> con);

    /* It stops the main thread which exchange information with the controller. */
    void stop();
  
    std::shared_ptr<Connection> connection;
  
  private:
    /* The main thread sends encoder's outputs and it gets control signals */
    void main(std::weak_ptr<Connection> con);

};

} // end namespace remy_robot_cotrol