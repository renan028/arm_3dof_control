#pragma once

// remy
#include <types.h>
#include <angle.h>
#include <connection.h>
#include <robot.h>

// std
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <chrono>

// Eigen
#include <Eigen/Geometry>

namespace remy_robot_control {

/*
 * This class reads information about the required trajectory of the robot 
 * manipulator from a file .in. It sends control signals over the connection 
 * and receives feedback from the robot.
*/
class Control {
  Connection connection;
  std::unique_ptr<Trajectory> trajectory;
  Robot model;
  std::function<Eigen::Vector3d(Eigen::Vector3d, double)> control_;

  public:
    Control(const std::string& input);
    ~Control() = default;

    /* This main method starts the controller. It periodically sends control 
     * signals to the robot.
    */
    void start();

    /* It gets the waypoints of the current trajectory
     \return waypoints
    */
    std::vector<Eigen::Vector4d> getWaypoints();

    /* Types of available Kinematics Control */
    enum class ControlType {
      feedfoward,
      analytical
    };

    /* It sets the type of control. Default is Feedfoward */
    void setControlStrategy(ControlType type);

    /* It solves the Kinematic Control problem for the 3-DoF (elbow) remy robot.
     * It computes the control signal for robot kinematics. There are some
     * control strategies, thus please refer to the documentation of the one
     * you want to use: 
     * \sa feedforwardControl
     * \sa pidControl
     * \sa analyticalControl
    */
    Eigen::Vector3d computeVelocityControl(const Eigen::Vector3d& joints, 
      double t);

  private:
    void readInput(const std::string& input);
  
  public:
    /*
     * The robot motion can be described by: qdot = u, where u is the velocity 
     * control signal (vector) applied to the motor drive of each joint.
     * We have: xdot = J * qdot = J * u, where J is the Jacobian
     * Then: u = inv(J) * v(t), where v(t) is the cartesian control signal.
     * e = xs(t) - x, where xs is the desired cartesian position, x the current
     * edot = xsdot - v(t), then we may choose a proportional plus feedforward
     * control law:
     * v(t) = xsdot + Le, where L = lambda * I (identity matrix), lambda > 0
     * Note that this is the Damping Method (DLS), thus if the robot starts at 
     * a position where J(q0) is singular and v is in its nullspace, it may
     * be impossible to find the inverse kinematics iteratively, and the control
     * fails. The solution is to introduce a term belonging to the nullsapce of
     * J:
     * u = inv(J) * v + (I - inv(J) * J) * q0, where q0 is an arbitrary joint 
     * velocities to avoid singularities.
     * \param joints the current robot joints
     * \param t current time (0 means start time)
    */
    Eigen::Vector3d feedforwardControl(const Eigen::Vector3d& joints, 
      double t);

    Eigen::Vector3d analyticalControl(const Eigen::Vector3d& joints, 
      double t);

    
};

} // end namespace remy_robot_control