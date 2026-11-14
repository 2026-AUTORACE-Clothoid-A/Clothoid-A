#include <gtest/gtest.h>

#include <cmath>

#include "limo_base/slow_topic.hpp"

using limo_base::ImuSlow;
using limo_base::OdomSlow;

static sensor_msgs::msg::Imu imu(int i)
{
  sensor_msgs::msg::Imu m;
  m.header.stamp.sec = 10;
  m.header.stamp.nanosec = static_cast<uint32_t>(i * 10000000);   // 100 Hz
  m.angular_velocity.z = i;            // 0, 1, 2, 3 ...
  m.angular_velocity.x = 2.0 * i;
  m.linear_acceleration.z = 9.8 + i;
  m.orientation.w = 1.0;
  m.orientation.z = 0.01 * i;
  return m;
}

TEST(ImuSlow, EveryFourthWithMeanRatesAndLastState)
{
  ImuSlow slow(4);
  sensor_msgs::msg::Imu out;
  int published = 0;
  for (int i = 0; i < 12; ++i) {
    if (slow.add(imu(i), out)) {
      ++published;
      // samples i-3 .. i
      EXPECT_DOUBLE_EQ(out.angular_velocity.z, i - 1.5);
      EXPECT_DOUBLE_EQ(out.angular_velocity.x, 2.0 * (i - 1.5));
      EXPECT_DOUBLE_EQ(out.linear_acceleration.z, 9.8 + i - 1.5);
      EXPECT_DOUBLE_EQ(out.orientation.z, 0.01 * i);
      EXPECT_EQ(out.header.stamp.nanosec, static_cast<uint32_t>(i * 10000000));
      EXPECT_EQ(i % 4, 3);
    }
  }
  EXPECT_EQ(published, 3);
}

TEST(ImuSlow, IntegralMatchesTheFullRate)
{
  // yaw from gyro * dt between stamps, as clothoid_parking's Tracker does
  ImuSlow slow(4);
  sensor_msgs::msg::Imu out;
  double full = 0.0, part = 0.0, t_last = 0.0;
  const double dt = 0.01;
  for (int i = 1; i <= 400; ++i) {
    sensor_msgs::msg::Imu m;
    m.angular_velocity.z = std::sin(i * 0.37);   // a fast wiggle a single sample would alias
    full += m.angular_velocity.z * dt;
    if (slow.add(m, out)) {
      part += out.angular_velocity.z * (i * dt - t_last);
      t_last = i * dt;
    }
  }
  EXPECT_NEAR(part, full, 1e-12);
}

TEST(ImuSlow, OneOrLessPassesEverything)
{
  for (int every : {0, 1}) {
    ImuSlow slow(every);
    sensor_msgs::msg::Imu out;
    for (int i = 0; i < 5; ++i) {
      ASSERT_TRUE(slow.add(imu(i), out));
      EXPECT_DOUBLE_EQ(out.angular_velocity.z, i);
    }
  }
}

TEST(OdomSlow, EverySecondWithMeanTwistAndLastPose)
{
  OdomSlow slow(2);
  nav_msgs::msg::Odometry in, out;
  int published = 0;
  for (int i = 0; i < 6; ++i) {
    in.twist.twist.linear.x = 0.1 * i;
    in.twist.twist.angular.z = -0.2 * i;
    in.pose.pose.position.x = 0.002 * i;
    in.header.stamp.nanosec = static_cast<uint32_t>(i * 20000000);
    if (slow.add(in, out)) {
      ++published;
      EXPECT_NEAR(out.twist.twist.linear.x, 0.1 * (i - 0.5), 1e-12);
      EXPECT_NEAR(out.twist.twist.angular.z, -0.2 * (i - 0.5), 1e-12);
      EXPECT_DOUBLE_EQ(out.pose.pose.position.x, 0.002 * i);
      EXPECT_EQ(out.header.stamp.nanosec, static_cast<uint32_t>(i * 20000000));
    }
  }
  EXPECT_EQ(published, 3);
}
