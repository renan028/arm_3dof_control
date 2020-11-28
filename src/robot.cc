#include <robot.h>
#include <utils.h>
#include <iostream>

namespace remy_robot_control {

Robot::Robot() {
  fk_ = [this](const Eigen::Vector3d& joints) {
    return fastForwardKinematics(joints);
  };

  ik_ = [this](double x, double y, double z) {
    return analyticalInverseKinematics(x, y, z);
  };

  double q1mM[2] = {joints_min[0], joints_max[0]};
  double q2mM[2] = {joints_min[1], joints_max[1]};
  double q3mM[2] = {joints_min[2], joints_max[2]};
  joints_ = std::make_unique<RemyJoints>(q1mM, q2mM, q3mM);
}

Robot::Robot(double q1, double q2, double q3) : Robot() {
  joints_->q1(q1);
  joints_->q1(q2);
  joints_->q1(q3);
}


void Robot::setFk(FkType type) {
  switch (type)
  {
    case FkType::fast:
      fk_ = [this](const Eigen::Vector3d& joints) {
        return fastForwardKinematics(joints);
      };
      break;
    
    default:
      fk_ = [this](const Eigen::Vector3d& joints) {
        return forwardKinematicsGeneric(joints);
      };
      break;
  }
}

void Robot::setIk(IkType type) {
  switch (type)
  {
    case IkType::transpose:
      ik_ = [this](double x, double y, double z) {
        return jacobTransposeIK(x, y, z);
      };
      break;

    case IkType::damped:
      ik_ = [this](double x, double y, double z) {
        return dampedIK(x, y, z);
      };
      break;
    
    default:
      ik_ = [this](double x, double y, double z) {
        return analyticalInverseKinematics(x, y, z);
      };
      break;
  }
}

Eigen::Vector3d Robot::forwardKinematics(const Eigen::Vector3d& joints) {
  return fk_(joints);
}

Eigen::Vector3d Robot::inverseKinematics(double x, double y, double z) {
  return ik_(x, y, z);
}

Eigen::Vector3d Robot::forwardKinematicsGeneric(
    const Eigen::Vector3d& joints) {
  auto T1 = DHToAffine(joints[0], M_PI_2, 10, 0);
  auto T2 = DHToAffine(joints[1], 0, 5, 0);
  auto T3 = DHToAffine(joints[2], 0, 5, 0);
  Eigen::Vector3d xyz = (T1*T2*T3).translation();
  return xyz;
}

Eigen::Vector3d Robot::fastForwardKinematics(
    const Eigen::Vector3d& joints) {
  double c1 = cos(joints[0]);
  double c2 = cos(joints[1]);
  double c23 = cos(joints[1] + joints[2]);
  double s1 = sin(joints[0]);
  double s2 = sin(joints[1]);
  double s23 = sin(joints[1] + joints[2]);

  double x = 5 * c1 * (2 + c2 + c23);
  double y = 5 * s1 * (2 + c2 + c23);
  double z = 5 * (s2 + s23);
  return {x, y, z};
}

JacobMP Robot::jacob(const Eigen::Vector3d& joints) {
  double s1 = sin(joints[0]);
  double s2 = sin(joints[1]);
  double c1 = cos(joints[0]);
  double c2 = cos(joints[1]);
  double c23 = cos(joints[1] + joints[2]);
  double s23 = sin(joints[1] + joints[2]);
  
  JacobMP m;
  m <<  - 5 * s1 * (c23 + c2 + 2), - 5 * c1 * (s23 + s2) , - 5 * s23 * c1,
        5 * c1 * (c23 + c2 + 2)  , -5 * s1 * (s23 + s2)  , - 5 * s23 * s1, 
        0                        , 5 * (c23 + c2)        , 5 * c23       ;
  return m;
}

Eigen::Vector3d Robot::jacobTransposeIK(double x, double y, double z, 
    const Eigen::Vector3d& q0, double error) {
  Eigen::Vector3d q = q0;
  Eigen::Vector3d xs = {x, y, z};
  int count = 0;
  while (count < 1e5) {
    auto x_ = forwardKinematics(q);
    auto dx = xs - x_;
    if (dx.norm() <= error) {
      return q;
    }
    auto J = jacob(q);
    auto Jt = J.transpose();
    q += Jt * dx * 0.01; 
    q[0] = joints_->q1(q[0]);
    q[1] = joints_->q1(q[1]);
    q[2] = joints_->q1(q[2]);
    ++count ;
  }
  auto x_ = forwardKinematics(q);
  return q;
}

Eigen::Vector3d Robot::dampedIK(double x, double y, double z, 
    const Eigen::Vector3d& q0, double error) {
    Eigen::Vector3d q = q0;
  Eigen::Vector3d xs = {x, y, z};
  int count = 0;
  while (count < 1e5) {
    auto x_ = forwardKinematics(q);
    auto dx = xs - x_;
    if (dx.norm() <= error) {
      return q;
    }
    auto J = jacob(q);
    auto Jt = J.transpose();
    auto JJt = J * Jt;
    auto L = Eigen::Matrix3d::Identity() * 0.1;
    q +=  Jt * (JJt + L).inverse() * dx; 
    q[0] = joints_->q1(q[0]);
    q[1] = joints_->q1(q[1]);
    q[2] = joints_->q1(q[2]);
    ++count ;
  }
  auto x_ = forwardKinematics(q);
  return q;
}

Eigen::Vector3d Robot::analyticalInverseKinematics(double x, double y, double z)
{
  Eigen::Vector3d q;
  
  // By the fast fk, we have:
  // double x = 5 * c1 * (2 + c2 + c23); [1]
  // double y = 5 * s1 * (2 + c2 + c23); [2]
  // double z = 5 * (s2 + s23); [3]
  // applying [2] / [1], we get theta1 = atan2(y, x). Note that we can only do 
  // that if (2 + c2 + c23) != 0, i.e., theta2 != - pi, and theta3 != 0 (singularity)
  // but that is always the case since -pi/2 < theta2 < pi/2
  q[0] = atan2(y, x);

  // Now theta2 and theta3 can be find by algebric manipulation of z and x 
  // equations or by using other approaches like Paden-Kahan Subproblems.
  // To solve it, we can do: [2]^2 + [3]^2 * (c1^2) to get:
  // q[2] = acos(0.5 * (2 + (1 / (25 * c1s)) * (xs + zs * c1s) - 4 * x / (5 * c1)));
  // but we should check cos(theta1) consistency first
  
  double c1 = cos(q[0]);
  double zs = z * z;
  double c1s = c1 * c1;
  double xs = x * x;
  
  if (abs(c1s) <= 1e-5) { // x ~= 0
    q[2] = acos(0.5 * (2 + (zs / (25))));
    double c3 = cos(q[2]);
    double s3 = sin(q[2]);
    q[1] =  asin((1 / (2 + 2 * c3)) * (z * (1 + c3) / 5 + 2 * s3));
  }
  else {
    q[2] = acos(0.5 * (2 + (1 / (25 * c1s)) * (xs + zs * c1s) - 4 * x / (5 * c1)));
    double c3 = cos(q[2]);
    double s3 = sin(q[2]);
    q[1] = asin((1 / (2 + 2 * c3)) * (z * (1 + c3) / 5 - x * s3 / (5 * c1) + 
      2 * s3));
  }
  return q;
}

Eigen::Vector3d Robot::getJoints() {
  Eigen::Vector3d joints;
  joints[0] = joints_->q1();
  joints[1] = joints_->q2();
  joints[2] = joints_->q3();
  return joints;
}

void Robot::update(const Eigen::Vector3d& u, double dt) {
  joints_->q1 += u[0] * dt; 
  joints_->q2 += u[1] * dt; 
  joints_->q2 += u[2] * dt; 
}


} // end namespace remy_robot_cotrol

