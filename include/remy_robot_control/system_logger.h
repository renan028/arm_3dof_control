// std
#include <string>
#include <fstream>
#include <iomanip>

// eigen
#include <Eigen/Geometry>

namespace remy_robot_control {

/** The SystemLogger is a simple logger to file csv. It handles the file creation,
 * open, close and stream. Not generic or fast, but just for this specific scenario.
 * The created file has the following format:
 * \rst
 * +------+---+---+---+----+----+----+--------+--------+--------+
 * | time | x | y | z | ux | uy | uz | theta1 | theta2 | theta3 |
 * +------+---+---+---+----+----+----+--------+--------+--------+
 * \endrst
 * Do not use it for production.
 */ 
class SystemLogger {
  std::ofstream file;
  public:
  /** The constructor requires a file name 
   * \param file_name the file name 
   */ 
    SystemLogger(const std::string& file_name) {
      file.open(file_name);
      file << "t,x,y,z,ux,uy,uz,t1,t2,t3\n";
    }

    ~SystemLogger() {
      file.close();
    }

    /** A method to save the simulated data in .csv file 
     * \param pos the current position
     * \param control the current control
     * \param q joints's angles
     * \param t time
    */
    void save(const Eigen::Vector3f& pos, const Eigen::Vector3f& control, 
        const Eigen::Vector3f& q, double t) {
      file << std::fixed << std::setprecision(3) << t << "," << pos[0] 
        << "," << pos[1] << "," << pos[2] << "," << control[0] << "," << 
        control[1] << "," << control[2] << "," << q[0] << "," << q[1] << "," << 
        q[2] << "\n";
}
};

} // end remy_robot_control