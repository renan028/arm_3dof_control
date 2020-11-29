#pragma once

// std
#include <memory>
#include <algorithm>
#include <vector>

// Eigen
#include <Eigen/Geometry>

// remy
#include <angle.h>
#include <trajectory.h>

namespace remy_robot_control {

typedef Eigen::Matrix<float, 6, 3> JacobM;
typedef Eigen::Matrix<float, 3, 3> JacobMP;
typedef Angle<float> Angled;

/** Types of Foward Kinematics */
enum class FkType {
  fast, ///< hardcoded and specifically for this case, it only computes the end-efector position
  generic ///< It uses the classic, and generic, transform multiplication with eigen
};

/** Types of Inverse Kinematics */
enum class IkType {
  transpose, ///< Iterative method that uses the jacobian transpose
  analytical, ///< It explores the translational part of the homogeneous transform
  damped ///< Damped least squares
};

/** Types of available Kinematics Control */
enum class ControlType {
  feedfoward, ///< Damping Least Square Control with extra term to avoid singularities
  analytical ///< The analytical solution
};

struct RemyJoints {
  Angled q1;
  Angled q2;
  Angled q3;
  RemyJoints(float q1mM[2], float q2mM[2], float q3mM[2]) {
    q1 = Angled(0, q1mM[0], q1mM[1]);
    q2 = Angled(0, q2mM[0], q2mM[1]);
    q3 = Angled(0, q3mM[0], q3mM[1]);
  }
};

} // end namespace remy_robot_cotrol