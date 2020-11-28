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
  Connection connection;
  std::thread thread;
  std::atomic<bool> stop_;
  Eigen::Vector3f control_signal;
  std::chrono::time_point<std::chrono::system_clock> clock;
  
  public:
    RobotSystem();
    ~RobotSystem();

    /* It creates a thread to start the robotic system. */
    void start();

    /* It stops the main thread which exchange information with the controller. */
    void stop();

  private:
    /* The main thread sends encoder's outputs and it gets control signals */
    void main();

};

} // end namespace remy_robot_cotrol