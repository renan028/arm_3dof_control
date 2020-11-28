#include <utils.h>

namespace remy_robot_control {

Eigen::Affine3f DHToAffine(float theta, float alpha, float link_length, 
    float offset) {
  float ct = cos(theta);
  float st = sin(theta);
  float ca = cos(alpha);
  float sa = sin(alpha);
  
  Eigen::Matrix3f R;
  R <<  ct, -st * ca,  st * sa, 
        st,  ct * ca, -ct * sa,
         0,   sa    ,  ca;
  Eigen::Vector3f v = {link_length * ct, link_length * st, offset};
  Eigen::Affine3f T = Eigen::Affine3f::Identity();
  T.linear() = R;
  T.translation() = v;
  return T;
}


float charToFloat(unsigned char c[4]) {
  float f = *(float*)c;
  return f;
}

unsigned char* floatToChar(float f) {
  unsigned char* s = new unsigned char[sizeof(float)];
  *(float*)(s) = f;
  return s;
}

Eigen::Vector3f uchar3ToEigen3f(const std::vector<unsigned char>& v_uchar) {
  Eigen::Vector3f u = Eigen::Vector3f::Zero();
  if (v_uchar.size() == 0)
    return u;
  unsigned char c1[4] = {v_uchar[0], v_uchar[1], v_uchar[2], v_uchar[3]};
  unsigned char c2[4] = {v_uchar[4], v_uchar[5], v_uchar[6], v_uchar[7]};
  unsigned char c3[4] = {v_uchar[8], v_uchar[9], v_uchar[10], v_uchar[11]};

  u[0] = charToFloat(c1);
  u[1] = charToFloat(c2);
  u[2] = charToFloat(c3);
  return u;
}

std::vector<unsigned char> eigen3fToUchar3(const Eigen::Vector3f& vec3) {
  std::vector<unsigned char> v_char;
  v_char.reserve(12);
  auto c1 = floatToChar(vec3[0]);
  auto c2 = floatToChar(vec3[1]);
  auto c3 = floatToChar(vec3[2]);
  for (auto c : {c1, c2, c3}) {
    v_char.push_back(c[0]);
    v_char.push_back(c[1]);
    v_char.push_back(c[2]);
    v_char.push_back(c[3]);
  }
  return v_char;
}

float encoderToJoint(int joint_int) {
  return 2 * M_PI * joint_int / 4096 - M_PI;
}

void mockEncoderPrecisionLost(float& joint) {
  int n = (int) (4096 * (joint + M_PI) / (2 * M_PI));
  joint = encoderToJoint(n);
}

void mockEncoderPrecisionLost(Eigen::Vector3f& joints) {
  mockEncoderPrecisionLost(joints[0]);
  mockEncoderPrecisionLost(joints[1]);
  mockEncoderPrecisionLost(joints[2]);
}

}
