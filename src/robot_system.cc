#include <robot_system.h>
#include <utils.h>
#include <fstream>
#include <iomanip>

namespace remy_robot_control {

RobotSystem::RobotSystem() :
  connection(std::make_shared<Connection>()),
  stop_(false),
  control_signal(Eigen::Vector3f::Zero()),
  clock(std::chrono::system_clock::now()),
  save_run(true),
  sleep_ms(1),
  encoder_resolution(4096)
{
}

RobotSystem::~RobotSystem(){
  stop();
};

void RobotSystem::setSettings(const RemySystemSettings& settings) {
  sleep_ms = (int)(1000 * (1.0 / (float)settings.frequency));
  encoder_resolution = settings.encoder_resolution;
}

void RobotSystem::setRobotSettings(const RemyRobotSettings& settings) {
  robot.setSettings(settings);
}

void RobotSystem::start(std::weak_ptr<Connection> con) {
  stop();
  stop_ = false;
  thread = std::thread(&RobotSystem::main, this, con);  
}

void RobotSystem::stop() {
  stop_ = true;
  if (thread.joinable()) thread.join();
}

void RobotSystem::main(std::weak_ptr<Connection> con) {
  connection->open();
  std::ofstream file;
  if (save_run) {
    file.open("out.csv");
    file << "t,x,y,z,ux,uy,uz,t1,t2,t3\n";
  }
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    else break;
  }

  clock = std::chrono::system_clock::now();
  double elapsed_time;
  while(auto conn = con.lock()) {
    if (!conn->isOpened()) break;
    if (stop_) {
      break;
    }
    auto new_clock = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_clock - clock;
    elapsed_time += diff.count();

    robot.update(control_signal, diff.count());
    auto q = robot.getJoints();
    
    if (save_run) {
      auto p = robot.forwardKinematics(q);
      save(p, control_signal, q, elapsed_time,file);
    }

    mockEncoderPrecisionLost(q, encoder_resolution);
    auto qc = eigen3fToUchar3(q);
    connection->send(qc);
    
    std::vector<unsigned char> control;
    conn->receive(control);
    control_signal = uchar3ToEigen3f(control);
    clock = new_clock;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
  if (save_run)
    file.close();
  connection->close();
}

void RobotSystem::save(const Eigen::Vector3f& pos, 
    const Eigen::Vector3f& control, const Eigen::Vector3f& q, double t, 
    std::ofstream& file) const {
  file << std::fixed << std::setprecision(3) << t << "," << pos[0] 
    << "," << pos[1] << "," << pos[2] << "," << control[0] << "," << 
    control[1] << "," << control[2] << "," << q[0] << "," << q[1] << "," << 
    q[2] << "\n";
}

} // end namespace remy_robot_control