// remy
#include <control.h>
#include <utils.h>

// std
#include <fstream>
#include <sstream>

namespace remy_robot_control {

Control::Control(const std::string& input) : 
    connection(std::make_shared<Connection>()),
    stop_(false),
    clock(std::chrono::system_clock::now()),
    sleep_ms(20) 
{
  readInput(input);
  setControlStrategy(ControlType::feedfoward);
}

Control::~Control() {
  stop();
}

void Control::setSettings(const RemyControlSettings& settings) {
  setControlStrategy(settings.control_type);
  sleep_ms = std::max(20, (int)(1000 * (1.0 / (float)settings.frequency)));
}

void Control::setRobotSettings(const RemyRobotSettings& settings) {
  model.setSettings(settings);
}

void Control::start(std::weak_ptr<Connection> con) {
  stop();
  stop_ = false;
  thread = std::thread(&Control::main, this, con);
}

void Control::stop() {
  stop_ = true;
  if (thread.joinable()) thread.join();
}

void Control::readInput(const std::string& input) {
  std::vector<Eigen::Vector4f> waypoints;
  std::ifstream file(input);
  std::string line;
  while(std::getline(file, line))
  {
    std::istringstream iss(line);
    float x, y, z, t;
    if (!(iss >> x >> y >> z >> t)) { break; }
    waypoints.push_back(Eigen::Vector4f(x, y, z, t)); 
  }
  trajectory = std::make_unique<Trajectory>(std::move(waypoints));
}

void Control::main(std::weak_ptr<Connection> con) {
  connection->open();
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    else break;
  }

  clock = std::chrono::system_clock::now();
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) break;
    if (stop_) {
      break;
    }
    std::vector<unsigned char> joints;
    conn->receive(joints);
    auto q = uchar3ToEigen3f(joints);
    
    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock;
    computeVelocityControl(q, diff.count());
    auto u_eigen = getControlSignal();
    auto u = eigen3fToUchar3(u_eigen);
    connection->send(u);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
  connection->close();
}

void Control::setControlStrategy(ControlType type) {
  switch (type)
  {
    case ControlType::feedfoward:
      control_ = [this](const Eigen::Vector3f& joints, float t) {
        return feedforwardControl(joints, t);
      };
      break;

    case ControlType::analytical:
      control_ = [this](const Eigen::Vector3f& joints, float t) {
        return analyticalControl(joints, t);
      };
      break;
    
    default:
      control_ = [this](const Eigen::Vector3f& joints, float t) {
        return feedforwardControl(joints, t);
      };
      break;
  }
}

std::vector<Eigen::Vector4f> Control::getWaypoints() const {
  return trajectory->waypoints;
}

Eigen::Vector3f Control::getControlSignal() const {
  return control_signal;
}

void Control::computeVelocityControl(const Eigen::Vector3f& joints, 
    float t) {
  control_signal = control_(joints, t);
}

Eigen::Vector3f Control::feedforwardControl(const Eigen::Vector3f& q,
    float t) {
  auto q0dot = Eigen::Vector3f(5, 5, 5);
  auto x = model.forwardKinematics(q);
  if (!trajectory->update(t)) {
    return Eigen::Vector3f::Zero();
  }
  auto dx = trajectory->x - x;
  auto v = trajectory->v + Eigen::Matrix3f::Identity() * dx;
  auto J = model.jacob(q);
  auto Jt = J.transpose();
  auto JJt = J * Jt;
  auto w = JJt.determinant();
  auto alpha = (w >= 0.001) ? 0 : 0.01 * (1 - w / 0.001) * (1 - w / 0.001);
  auto L = Eigen::Matrix3f::Identity() * alpha;
  auto Ji = Jt * (JJt + L).inverse();
  return Ji * v + (Eigen::Matrix3f::Identity() - Ji * J) * q0dot;
}

Eigen::Vector3f Control::analyticalControl(const Eigen::Vector3f& q,
    float t) {
  float dt = (float)sleep_ms / 1000;
  if (!trajectory->update(t)) {
    return Eigen::Vector3f::Zero();
  }
  auto xd = trajectory->x;
  auto qd = model.inverseKinematics(xd[0], xd[1], xd[2]);
  auto dq = qd - q;
  return dq / dt; 
}

} // end namespace remy_robot_control