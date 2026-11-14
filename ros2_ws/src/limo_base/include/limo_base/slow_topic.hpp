// Slow copies of /imu and /odom for the Python nodes (docs/requests/imu_odom_25hz.md).
//
// Every message a Python node receives costs the rclpy executor more than the node's own work, and
// every node only needs a few samples per camera frame. ImuSlow / OdomSlow pass every n-th message:
//   - rates (Imu angular_velocity and linear_acceleration, Odometry twist) are the mean of the n
//     messages, so a node that integrates rate * dt between stamps gets what it would have
//     integrated from all n (no aliasing of a short turn or bump);
//   - states (orientation, pose, header.stamp, frame ids, covariances) are those of the n-th.
// n <= 1 passes every message unchanged.
#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace limo_base
{

struct Vec3Sum
{
  double x = 0.0, y = 0.0, z = 0.0;

  template<class V>
  void add(const V & v) { x += v.x; y += v.y; z += v.z; }

  template<class V>
  void mean_into(V & v, int n) const { v.x = x / n; v.y = y / n; v.z = z / n; }
};

class ImuSlow
{
public:
  explicit ImuSlow(int every) : every_(every) {}

  // true when `out` holds the slow message for this input
  bool add(const sensor_msgs::msg::Imu & in, sensor_msgs::msg::Imu & out)
  {
    if (every_ <= 1) {
      out = in;
      return true;
    }
    gyro_.add(in.angular_velocity);
    accel_.add(in.linear_acceleration);
    if (++count_ < every_) {
      return false;
    }
    out = in;
    gyro_.mean_into(out.angular_velocity, count_);
    accel_.mean_into(out.linear_acceleration, count_);
    count_ = 0;
    gyro_ = Vec3Sum();
    accel_ = Vec3Sum();
    return true;
  }

private:
  int every_;
  int count_ = 0;
  Vec3Sum gyro_, accel_;
};

class OdomSlow
{
public:
  explicit OdomSlow(int every) : every_(every) {}

  bool add(const nav_msgs::msg::Odometry & in, nav_msgs::msg::Odometry & out)
  {
    if (every_ <= 1) {
      out = in;
      return true;
    }
    linear_.add(in.twist.twist.linear);
    angular_.add(in.twist.twist.angular);
    if (++count_ < every_) {
      return false;
    }
    out = in;
    linear_.mean_into(out.twist.twist.linear, count_);
    angular_.mean_into(out.twist.twist.angular, count_);
    count_ = 0;
    linear_ = Vec3Sum();
    angular_ = Vec3Sum();
    return true;
  }

private:
  int every_;
  int count_ = 0;
  Vec3Sum linear_, angular_;
};

}  // namespace limo_base
