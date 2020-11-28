#pragma once

// remy
#include <types.h>
#include <angle.h>

// std
#include <vector>
#include <functional>
#include <memory>

// Eigen
#include <Eigen/Geometry>

namespace remy_robot_control {

/* The Class Robot calculates the forward and inverse kinematics of a R-RR 
  (elbow)robot with the following DH parameters:
Link No | Twist(alpha) | Link length(a) | Link offset(d) | Joint angle (theta)
   0    |     0        |       0        |     -          |
   1    |    pi/2      |       10       |     0          |   theta1
   2    |     0        |       5        |     0          |   theta2
   3    |     0        |       5        |     0          |   theta3

And the following joint limits:
Joint No | Min limit | Max limit 
  1      |    -pi    |    pi
  2      |    -pi/2  |    pi/2
  3      |    -pi    |    pi
 */
class Robot {
  
  std::function<Eigen::Vector3f(const Eigen::Vector3f& joints)> fk_;
  std::function<Eigen::Vector3f(float, float, float)> ik_;
  // TODO: get from file parameters
  Eigen::Vector3f joints_min = {- M_PI, - M_PI_2, - M_PI};
  Eigen::Vector3f joints_max = {M_PI, M_PI_2, M_PI};
  std::unique_ptr<RemyJoints> joints_;

  public:
    Robot();
    Robot(float q1, float q2, float q3);
    ~Robot() = default;

    /*  It returns the foward kinematics for the R-RR robot. 
      * \param joints the current values for robot joints
      * \return the end-effector position (not pose)
    */
    Eigen::Vector3f forwardKinematics(const Eigen::Vector3f& joints);
    
    /*  It returns the inverse kinematics for the R-RR (elbow) robot. 
      * \param x
      * \param y
      * \param z
      * \return the end-effector position (not pose)
    */
    Eigen::Vector3f inverseKinematics(float x, float y, float z);

    /* Types of Foward Kinematics*/
    enum class FkType {
      fast, // hardcoded and specifically for this case, it only computes the end-efector position
      generic // It uses the classic, and generic, transform multiplication with eigen
    };

    /* Types of Inverse Kinematics*/
    enum class IkType {
      transpose, // Iterative method that uses the jacobian transpose
      analytical, // It explores the translational part of the homogeneous transform
      damped // Damped least squares
    };

    /*  It changes the foward kinematics method 
      * \param type \sa FKType
    */
    void setFk(FkType type);

    /*  It changes the inverse kinematics method 
     * \param type \sa IKType
    */
    void setIk(IkType type);

    /* It returns the robot joints. This is not the one that is sent (no connection 
     * here)
     * \return robot's joints
    */
    Eigen::Vector3f getJoints();

    

    /* It is the system model qdot = u, thus q = u * dt. It integrates the 
     * input control.
     * \param u the input control
     * \param dt the integration step
    */
    void update(const Eigen::Vector3f& u, float dt);
  
  private: 


    /* The position was analytically calculated and the result was pasted here
     * \param joints the current values for robot joints
    */
    Eigen::Vector3f fastForwardKinematics(const Eigen::Vector3f& joints);

    /* The forward kinematics is computed by the multiplication of the 
     * homogeneous transformations T0*T1*T2*T3, which can be formed directly 
     * from the DH parameters.
     * T = cos(theta) | -sin(theta)*cos(alpha) | sin(theta)sin(alpha)  | a*cos(theta)
       *   sin(theta) |  cos(theta)*cos(alpha) | -cos(theta)sin(alpha) | a*sin(theta)
       *   0          |  sin(alpha)            | cos(alpha)            | d
       *   0          |  0                     | 0                     | 1
     * \param joints the current values for robot joints
    */
    Eigen::Vector3f forwardKinematicsGeneric(const Eigen::Vector3f& joints);

    /* It uses the Jacobian to iteratively compute the inverse kinematics.
     * 1) e = Xg - X
     * 2) J(joints)
     * 3) q = q + J^t * e * alpha (one could use the best alpha, but fixed here)
     * \param x 
     * \param y
     * \param z 
     * \param q0 initial guess for joints
     * \param error the desired error
     * \return the joints to reach (x, y, z)
    */
   // TODO: this should be robot config
    Eigen::Vector3f jacobTransposeIK(float x, float y, float z, 
      const Eigen::Vector3f& q0 = {0, 0, 0}, float error = 1e-3);

    /* It uses the Damped least squares method to iteratively compute the 
     * inverse kinematics.
     * 1) e = Xg - X
     * 2) J(joints)
     * 3) q = q + J^t * inv(J * J^t + lambda^2 * I) * e
     * \param x 
     * \param y
     * \param z 
     * \param q0 initial guess for joints
     * \param error the desired error
     * \return the joints to reach (x, y, z)
    */
    Eigen::Vector3f dampedIK(float x, float y, float z, 
      const Eigen::Vector3f& q0 = {0, 0, 0}, float error = 1e-3);

    /* It computes the analytical solution for inverse kinematics, for this 
     * specific case by algebric manipulation of the translation vector, and using 
     * trigonometric equalities. Note that we may have more than one solution
     * for some positions.
     * \param x 
     * \param y
     * \param z 
     * \return the joints to reach (x, y, z)
    */
    Eigen::Vector3f analyticalInverseKinematics(float x, float y, float z);

  public:
    /* It calculates the Jacobian (dP/dtheta only translation) 
    * \param joints the current values for robot joints
    * \return the jacobian as an eigen matrix (3x3)
    */
    JacobMP jacob(const Eigen::Vector3f& joints);
};

} // end namespace remy_robot_cotrol