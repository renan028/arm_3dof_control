#include <utils.h>

namespace remy_robot_control {

Eigen::Affine3d DHToAffine(double theta, double alpha, double link_length, 
    double offset) {
  double ct = cos(theta);
  double st = sin(theta);
  double ca = cos(alpha);
  double sa = sin(alpha);
  
  Eigen::Matrix3d R;
  R <<  ct, -st * ca,  st * sa, 
        st,  ct * ca, -ct * sa,
         0,   sa    ,  ca;
  Eigen::Vector3d v = {link_length * ct, link_length * st, offset};
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R;
  T.translation() = v;
  return T;
}

}
