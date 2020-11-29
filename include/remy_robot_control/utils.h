#pragma once

// remy
#include <types.h>

// std
#include <vector>

// Eigen
#include <Eigen/Geometry>

// 3rd
#include <json.hpp>
namespace remy_robot_control {

/** A function to compute the Homogenous Transform given the DH parameters for 
  Revolution joints.
  \param theta theta angle for revolution joints
  \param alpha angle between \f$Z_{i-1}\f$ and \f$Z_i\f$ wrt to the normal between them
  \param link_length length of the link
  \param offset offset between \f$X_{i-1}\f$ and \f$X_i\f$
*/
Eigen::Affine3f DHToAffine(float theta, float alpha, float link_length, 
    float offset = 0);

/** Explicit pointer conversion char to float 
 * \param buffer char[4]
*/
float charToFloat(unsigned char buffer[4]);

/** Explicit pointer conversion float to char
 * \param f float
*/
unsigned char* floatToChar(float f);

/** It converts the vector<uchar> to Eigen::Vector3f
 * \param v_uchar vector<uchar> dimension 12 (3*4)
 * \return Eigen::Vector3f
 */
Eigen::Vector3f uchar3ToEigen3f(const std::vector<unsigned char>& v_uchar);

/** It converts the Eigen::Vector3f to vector<uchar>
 * \param vec3 Eigen::Vector3f 
 * \return vector<uchar> dimension 12 (3*4)
 */
std::vector<unsigned char> eigen3fToUchar3(const Eigen::Vector3f& vec3);

/** Encoder output to joint \f$[-\pi, \pi]\f$. The encoder is 12-bit precision.
 * \param joint_int
 * \return joint_float
*/
float encoderToJoint(int joint_int);

/** It mocks the precision lost 12-bit of the encoder and it outputs the new 
 * joint \f$[-\pi, \pi]\f$. Inline modification. 
 * \param joint
*/
void mockEncoderPrecisionLost(float& joint);
void mockEncoderPrecisionLost(Eigen::Vector3f& joints);
using json = nlohmann::json;

/** Function to get a value from an attribute given the json file and field
 * \param j the jason file
 * \param field the field to get attribute from
 * \param att the attribute
 * \return value
 */
template<class T>
T parseJsonFieldAtt(const json& j, const std::string& field, 
    const std::string& att) {
  return j[field][att];
}

/** Function to retrieve robot settings from json file */ 
RemyRobotSettings parseRobotSetting(const json& j);

/** Function to retrieve system settings from json file */ 
RemySystemSettings parseSystemSetting(const json& j);

/** Function to retrieve control settings from json file */ 
RemyControlSettings parseControlSetting(const json& j);

}
