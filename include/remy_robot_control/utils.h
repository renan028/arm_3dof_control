#pragma once

#include <Eigen/Geometry>
#include <vector>

namespace remy_robot_control {

/* A function to compute the Homogenous Transform given the DH parameters for 
  Revolution joints.
  \param theta theta angle for revolution joints
  \param alpha angle between Zi-1 and Zi wrt to the normal between them
  \param link_length length of the link
  \param offset offset between Xi-1 and Xi
*/
Eigen::Affine3f DHToAffine(float theta, float alpha, float link_length, 
    float offset = 0);

/* Explicit pointer conversion char to float 
 * \param buffer char[4]
*/
float charToFloat(unsigned char buffer[4]);

/* Explicit pointer conversion float to char
 * \param f float
*/
unsigned char* floatToChar(float f);

/* It converts the vector<uchar> to Eigen::Vector3f
 * \param v_uchar vector<uchar> dimension 12 (3*4)
 * \return Eigen::Vector3f
 */
Eigen::Vector3f uchar3ToEigen3f(const std::vector<unsigned char>& v_uchar);

/* It converts the Eigen::Vector3f to vector<uchar>
 * \param vec3 Eigen::Vector3f 
 * \return vector<uchar> dimension 12 (3*4)
 */
std::vector<unsigned char> eigen3fToUchar3(const Eigen::Vector3f& vec3);

}
