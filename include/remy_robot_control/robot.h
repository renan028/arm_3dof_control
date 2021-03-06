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

/** The Class Robot calculates the forward and inverse kinematics of a R-RR 
 * (elbow)robot with the following DH parameters:
 * \rst
 * +-------+-------+--------+--------+-------+
 * | Link  | Twist | Length | Offset | angle |
 * +-------+-------+--------+--------+-------+
 * |   0   |   0   |   0    |   -    |   -   |
 * +-------+-------+--------+--------+-------+
 * |   1   |  pi/2 |   10   |   -    |   t1  |
 * +-------+-------+--------+--------+-------+
 * |   2   |   0   |   5    |   -    |   t2  |
 * +-------+-------+--------+--------+-------+
 * |   3   |   0   |   5    |   -    |   t3  |
 * +-------+-------+--------+--------+-------+
 * \endrst
 *
 * And the following joint limits:
 * \rst
 * +-------+-------+-------+
 * | Joint |  Min  |  Max  |
 * +-------+-------+-------+
 * |   1   |  -pi  |   pi  |
 * +-------+-------+-------+
 * |   2   | -pi/2 | pi/2  |
 * +-------+-------+-------+
 * |   3   |  -pi  |   pi  |
 * +-------+-------+-------+
 * \endrst
 */
class Robot {
  
  std::function<Eigen::Vector3f(const Eigen::Vector3f& joints)> fk_;
  std::function<Eigen::Vector3f(float, float, float)> ik_;
  std::unique_ptr<RemyJoints> joints_;

  public:
    Robot();
    Robot(float q1, float q2, float q3);
    ~Robot() = default;

    /** It sets the robot settings
     * \param settings \sa RemyRobotSettings
     */ 
    void setSettings(const RemyRobotSettings& settings);

    /**  It returns the foward kinematics for the R-RR robot. 
      * \param joints the current values for robot joints
      * \return the end-effector position (not pose)
    */
    Eigen::Vector3f forwardKinematics(const Eigen::Vector3f& joints);
    
    /**  It returns the inverse kinematics for the R-RR (elbow) robot. 
      * \param x
      * \param y
      * \param z
      * \return the end-effector position (not pose)
    */
    Eigen::Vector3f inverseKinematics(float x, float y, float z);

    /**  It changes the foward kinematics method 
      * \param type \sa FKType
    */
    void setFk(FkType type = FkType::fast);

    /**  It changes the inverse kinematics method 
     * \param type \sa IKType
    */
    void setIk(IkType type = IkType::analytical);

    /** It returns the robot joints. This is not the one that is sent (no connection 
     * here)
     * \return robot's joints
    */
    Eigen::Vector3f getJoints();

    /** It is the system model \f$ \dot{q}=u \f$, thus \f$ q=u*dt \f$. It 
     * integrates the input control.
     * \param u the input control
     * \param dt the integration step
    */
    void update(const Eigen::Vector3f& u, float dt);
  
  private: 


    /** The position was analytically calculated and the result was pasted here
     * \param joints the current values for robot joints
    */
    Eigen::Vector3f fastForwardKinematics(const Eigen::Vector3f& joints);

    /** The forward kinematics is computed by the multiplication of the 
     * homogeneous transformations \f$ T_0*T_1*T_2*T_3 \f$, which can be formed 
     * directly from the DH parameters. T =
     * \rst
     * +--------------+------------------------+-----------------------+--------------+
     * |  cos(theta)  | -sin(theta)*cos(alpha) | sin(theta)sin(alpha)  | a*cos(theta) |
     * +--------------+------------------------+-----------------------+--------------+
     * |  sin(theta)  | cos(theta)*cos(alpha)  | -cos(theta)sin(alpha) | a*sin(theta) |
     * +--------------+------------------------+-----------------------+--------------+
     * |      0       |       sin(alpha)       |      cos(alpha)       |      d       |
     * +--------------+------------------------+-----------------------+--------------+
     * |      0       |           0            |          0            |      1       |
     * +--------------+------------------------+-----------------------+--------------+
     * \endrst
     * \param joints the current values for robot joints
    */
    Eigen::Vector3f forwardKinematicsGeneric(const Eigen::Vector3f& joints);

    /** It uses the Jacobian to iteratively compute the inverse kinematics. The
     * algorithm' steps are: 1) \f$ e = x_g - x \f$, 2) compute \f$ J(q) \f$,
     * 3) \f$ q = q + J^t * e * \alpha \f$ 
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

    /** It uses the Damped least squares method to iteratively compute the 
     * inverse kinematics. The algorithm' steps are: 1) \f$ e = x_g - x \f$, 
     * 2) compute \f$ J(q) \f$,
     * 3) \f$ q = q + J^t * (J * J^t + \lambda^2 * I)^{-1} * e \f$
     * \param x 
     * \param y
     * \param z 
     * \param q0 initial guess for joints
     * \param error the desired error
     * \return the joints to reach (x, y, z)
    */
    Eigen::Vector3f dampedIK(float x, float y, float z, 
      const Eigen::Vector3f& q0 = {0, 0, 0}, float error = 1e-3);

    /** It computes the analytical solution for inverse kinematics, for this 
     * specific case by algebric manipulation of the translation vector, and using 
     * trigonometric equalities. Note that we may have more than one solution
     * for some positions.
     * \param x 
     * \param y
     * \param z 
     * \return the joints to reach (x, y, z)
    */
    Eigen::Vector3f analyticalIK(float x, float y, float z);

  public:
    /** It calculates the Jacobian (\f$\frac{dP}{d\theta}\f$ only translation) 
    * \param joints the current values for robot joints
    * \return the jacobian as an eigen matrix (3x3)
    */
    JacobMP jacob(const Eigen::Vector3f& joints);
};

} // end namespace remy_robot_cotrol