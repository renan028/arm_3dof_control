#pragma once

#include <Eigen/Geometry>

namespace remy_robot_control {

/* A function to compute the Homogenous Transform given the DH parameters for 
  Revolution joints.
  \param theta theta angle for revolution joints
  \param alpha angle between Zi-1 and Zi wrt to the normal between them
  \param link_length length of the link
  \param offset offset between Xi-1 and Xi
*/
Eigen::Affine3d DHToAffine(double theta, double alpha, double link_length, 
    double offset = 0);

}
