// remy
#include <control.h>

// std
#include <fstream>
#include <sstream>
#include <iostream>

namespace remy_robot_control {

Control::Control(const std::string& input) {
  readInput(input);
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

std::vector<Eigen::Vector4d> Control::getWaypoints() {
  return trajectory->waypoints;
}

Eigen::Vector3d Control::computeVelocityControl(const Eigen::Vector3d& q,
    double t) {
  auto x = model.forwardKinematics(q);
  trajectory->update(t);
  auto v = trajectory->v + Eigen::Matrix3d::Identity() * (trajectory->x - x);
  auto J = model.jacob(q);
  auto Jt = J.transpose();
  auto JJt = J * Jt;
  auto w = JJt.determinant();
  auto alpha = (w >= 0.1) ? 0 : 10 * (1 - w/0.1) * (1 - w/0.1);
  auto L = Eigen::Matrix3d::Identity() * alpha;
  return Jt * (JJt + L).inverse() * v; 
}

} // end namespace remy_robot_control