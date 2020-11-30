#pragma once

// remy
#include <robot.h>
#include <connection.h>
#include <system_logger.h>

// std
#include <thread>
#include <atomic>

namespace remy_robot_control {
  
class RobotSystem {
  Robot robot;
  std::thread thread;
  std::atomic<bool> stop_;
  Eigen::Vector3f control_signal;
  std::chrono::time_point<std::chrono::system_clock> clock;
  int sleep_ms;
  int encoder_resolution;
  std::unique_ptr<SystemLogger> logger;
  
  public:
    RobotSystem();
    ~RobotSystem();
    
    /** It sets the system settings
     * \param settings \sa RemySystemSettings
    */ 
    void setSettings(const RemySystemSettings& settings);

    /** It changes the robot model settings by a given new one
     * \param settings new settings
     */
    void setRobotSettings(const RemyRobotSettings& settings);

    /** It creates a thread to start the robotic system. */
    void start(std::weak_ptr<Connection> con);

    /** It stops the main thread which exchange information with the controller. */
    void stop();

    
    std::shared_ptr<Connection> connection;
    bool save_run;
  
  private:
    /** The main thread sends encoder's outputs and it gets control signals */
    void main(std::weak_ptr<Connection> con);
};

} // end namespace remy_robot_cotrol