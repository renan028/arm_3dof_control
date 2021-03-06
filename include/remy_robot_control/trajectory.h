#pragma once

#include <Eigen/Geometry>
#include <vector>
#include <algorithm>

/** This is a very simple way to make a piecewise-linear trajectory that computes
 * position and velocity (constant) given time and waypoints. 
 * In a general case, this should be a 3D-Spline, and we would be able to 
 * choose a velocity profile, with accelerations, for instance a trapezoidal 
 * profile.
 */
class Trajectory {
  public: 
    Eigen::Vector3f x;
    Eigen::Vector3f v;
    std::vector<Eigen::Vector4f> waypoints;
    std::vector<float> times;

    Trajectory(const std::vector<Eigen::Vector4f>& _waypoints) : 
        waypoints(_waypoints) {
      times.reserve(waypoints.size());
      for (auto waypoint : waypoints) {
        times.push_back(waypoint[3]);
      }
    }
    
    bool update(float t) {
      auto it = std::lower_bound(times.begin(), times.end(), t);
      if (it == times.end()) return false;
      
      size_t index = it - times.begin();
      index = (index >= times.size()) ? times.size() - 1 : index;
      auto t0 = (index == 0) ? 0 : times[index - 1];
      auto tf = times[index];
      auto x0 = (index == 0) ? Eigen::Vector3f::Zero() : 
        Eigen::Vector3f(waypoints[index - 1].head(3));
      auto xf = Eigen::Vector3f(waypoints[index].head(3));
      
      // x(t) = x0 + v(t-t0)
      // v = (xf - x0) / (tf - t0)
      if (tf == t0) {
        v = Eigen::Vector3f::Zero();
        x = xf;
        return true;
      }
      v = (xf - x0) / (tf - t0);
      x = x0 + v * (t - t0);
      return true;
    }
};