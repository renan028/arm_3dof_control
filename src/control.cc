// remy
#include <control.h>

// std
#include <fstream>
#include <sstream>
#include <iostream>

namespace remy_robot_control {

Control::Control(const std::string& input) {
  readInput(input);
  setControlStrategy(ControlType::feedfoward);
}

void Control::readInput(const std::string& input) {
  std::vector<Eigen::Vector4d> waypoints;
  std::ifstream file(input);
  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    double x, y, z, t;
    if (!(iss >> x >> y >> z >> t)) { break; }
    waypoints.push_back(Eigen::Vector4d(x, y, z, t)); 
  }
  trajectory = std::make_unique<Trajectory>(std::move(waypoints));
}

void Control::setControlStrategy(ControlType type) {
  switch (type)
  {
    case ControlType::feedfoward:
      control_ = [this](const Eigen::Vector3d& joints, double t) {
        return feedforwardControl(joints, t);
      };
      break;

    case ControlType::analytical:
      control_ = [this](const Eigen::Vector3d& joints, double t) {
        return analyticalControl(joints, t);
      };
      break;
    
    default:
      control_ = [this](const Eigen::Vector3d& joints, double t) {
        return feedforwardControl(joints, t);
      };
      break;
  }
}

std::vector<Eigen::Vector4d> Control::getWaypoints() {
  return trajectory->waypoints;
}

Eigen::Vector3d Control::computeVelocityControl(const Eigen::Vector3d& joints, 
    double t) {
  return control_(joints, t);
}

Eigen::Vector3d Control::feedforwardControl(const Eigen::Vector3d& q,
    double t) {
  auto q0dot = Eigen::Vector3d(0.1, 0.1, 0.1);
  auto x = model.forwardKinematics(q);
  if (!trajectory->update(t)) {
    return Eigen::Vector3d::Zero();
  }
  auto dx = trajectory->x - x;
  auto v = trajectory->v + Eigen::Matrix3d::Identity() * dx;
  auto J = model.jacob(q);
  auto Jt = J.transpose();
  auto JJt = J * Jt;
  auto w = JJt.determinant();
  auto alpha = (w >= 0.001) ? 0 : 1 * (1 - w / 0.001) * (1 - w / 0.001);
  auto L = Eigen::Matrix3d::Identity() * alpha;
  auto Ji = Jt * (JJt + L).inverse();
  return Ji * v + (Eigen::Matrix3d::Identity() - Ji * J) * q0dot; 
}

Eigen::Vector3d Control::analyticalControl(const Eigen::Vector3d& q,
    double t) {
  // TODO: change dt here
  auto dt = 0.02;
  if (!trajectory->update(t)) {
    return Eigen::Vector3d::Zero();
  }
  auto xd = trajectory->x;
  auto qd = model.inverseKinematics(xd[0], xd[1], xd[2]);
  auto dq = qd - q;
  return dq / dt; 
}

} // end namespace remy_robot_control