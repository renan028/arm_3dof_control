#pragma once

namespace remy_robot_control {

/* A custom class to handle angles properly (-PI, PI). One may choose the angle
 * Limits.
*/
template <class T = float>
class Angle {
  T value_;
  T min_;
  T max_;
  
  public: 
    Angle(T value = T{}, T min = static_cast<T>(-M_PI), 
      T max = static_cast<T>(M_PI)) : value_(value), max_(max), min_(min)
    {
      norm();
    }
    
    T operator()(void){
      return value_;
    }

    T operator()(const T& new_value){
      value_ = new_value;
      norm();
      return value_;
    }

    Angle& operator+=(const Angle& rhs)
    { 
      value_ += rhs.value_;
      norm();                          
      return *this;
    }

    friend Angle operator+(Angle lhs, const Angle& rhs)
    {
      lhs.value += rhs.value;
      norm();
      return lhs;
    }

    Angle& operator=(const Angle& other) {
      value_ = other.value_;
      min_ = other.min_;
      max_ = other.max_;
    }

    Angle& operator=(const T& other) {
      value_ = other;
      norm();
    }

  private:
    void norm() {
      value_ = fmod(value_, 2 * M_PI);
      value_ += (value_ > M_PI) ? - 2 * M_PI : 0;
      value_ += (value_ < - M_PI) ? 2 * M_PI : 0;
      value_ = std::min(std::max(value_, min_), max_);
    }
};

} // end namespace remy_robot_cotrol