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