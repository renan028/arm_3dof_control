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
#include <thread>
#include <atomic>

// Eigen
#include <Eigen/Geometry>

namespace remy_robot_control {

/**
 * This class reads information about the required trajectory of the robot 
 * manipulator from a file .in. It sends control signals over the connection 
 * and receives feedback from the robot.
*/
class Control {
  std::unique_ptr<Trajectory> trajectory;
  Robot model;
  std::function<Eigen::Vector3f(Eigen::Vector3f, float)> control_;
  Eigen::Vector3f control_signal;
  std::thread thread;
  std::atomic<bool> stop_;
  std::chrono::time_point<std::chrono::system_clock> clock;

  public:
    Control(const std::string& input);
    ~Control();

    /** It creates a thread to control the arm. */
    void start(std::weak_ptr<Connection> con);

    /** It stops the thread which controls the arm. */
    void stop();

    /** It gets the waypoints of the current trajectory
     \return waypoints
    */
    std::vector<Eigen::Vector4f> getWaypoints() const;

    /** It gets the current control signal
     \return control_signal
    */
    Eigen::Vector3f getControlSignal() const;

    /** Types of available Kinematics Control */
    enum class ControlType {
      feedfoward, ///< Damping Least Square Control with extra term to avoid singularities
      analytical ///< The analytical solution
    };

    /** It sets the type of control. Default is Feedfoward */
    void setControlStrategy(ControlType type);

    /** It solves the Kinematic Control problem for the 3-DoF (elbow) remy robot.
     * It computes the control signal for robot kinematics. There are some
     * control strategies, thus please refer to the documentation of the one
     * you want to use: 
     * \sa feedforwardControl
     * \sa analyticalControl
    */
    void computeVelocityControl(const Eigen::Vector3f& joints, float t);

    std::shared_ptr<Connection> connection;

  private:
    /** It read the input file 
     * \param input absolute file path
    */
    void readInput(const std::string& input);

    /** This is the main thread of the controller. It periodically sends control 
     * signals to the robot.
    */
    void main(std::weak_ptr<Connection> con);
  
    /** The robot motion can be described by: \f$\dot{q} = u\f$, where \f$u\f$ 
     * is the velocity control signal (vector) applied to the motor drive of each joint. \n
     * We have: \f$\dot{x} = J * \dot{q} = J * u\f$, where \f$J\f$ is the Jacobian \n
     * Then: \f$u = J^{-1} * v(t)\f$, where \f$v(t)\f$ is the cartesian control signal. \n
     * \f$e = x_s(t) - x\f$, where \f$x_s\f$ is the desired cartesian position, 
     * \f$x\f$ the current position. \n  
     * \f$\dot{e} = \dot{x_s} - v(t)\f$, then we may choose a proportional plus 
     * feedforward control law: \n
     * \f$ v(t) = \dot{x_s} + \Lambda * e \f$, where \f$L = \lambda * I\f$ 
     * (identity matrix), \f$\lambda > 0\f$ \n
     * Note that this is the Damping Method (DLS), thus if the robot starts at 
     * a position where \f$J(q0)\f$ is singular and \f$v\f$ is in its nullspace, 
     * it may be impossible to find the inverse kinematics iteratively, and the 
     * control fails. The solution is to introduce a term belonging to the 
     * nullsapce of \f$J\f$: \n
     * \f$u = J^{-1} * v + (I - J^{-1} * J) * q_0\f$, where \f$q_0\f$ is an 
     * arbitrary joint velocities to avoid singularities.
     * \param joints the current robot joints
     * \param t current time (0 means start time)
    */
    Eigen::Vector3f feedforwardControl(const Eigen::Vector3f& joints, 
      float t);

    /** The Analytical Control solution uses the analytical inverse kinematics
     * and it applies \f$u = \frac{dq}{dt} = (q_s - q) / dt\f$
     * \param joints the current robot joints
     * \param t current time (0 means start time)
    */
    Eigen::Vector3f analyticalControl(const Eigen::Vector3f& joints, 
      float t);

    
};

} // end namespace remy_robot_control