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

  public:
    Control(const std::string& input);
    ~Control() = default;
    void start();

    /* It gets the waypoints of the current trajectory
     \return waypoints
    */
    std::vector<Eigen::Vector4d> getWaypoints();

  private:
    void readInput(const std::string& input);
  
  public:
    /*
     * It solves the Kinematic Control problem for the 3-DoF (elbow) remy robot.
     * The robot motion can be described by: qdot = u, where u is the velocity 
     * control signal (vetor) applied to the motor drive of each joint.
     * We have: xdot = J * qdot = J * u, where J is the Jacobian
     * Then: u = inv(J) * v(t), where v(t) is the cartesian control signal.
     * e = xs(t) - x
     * edot = xsdot - v(t), then we may choose a proportional plus feedforward
     * control law:
     * v(t) = xsdot + Le, where L = lambda * I (identity matrix), lambda > 0
     * \param joints the current robot joints
     * \param t current time (0 means start time)
    */
    Eigen::Vector3d computeVelocityControl(const Eigen::Vector3d& joints, 
      double t);

    
};

} // end namespace remy_robot_control