#pragma once
#ifndef AMR_SAFETY_CORE__SAFETY_POLICY_HPP_
#define AMR_SAFETY_CORE__SAFETY_POLICY_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>

namespace amr_safety_core
{

inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// First-order low-pass filter: alpha in [0,1], alpha=1 means no filtering
inline double lpf_update(double current, double target, double alpha)
{
  return current + alpha * (target - current);
}

inline geometry_msgs::msg::Twist apply_scale(
  const geometry_msgs::msg::Twist & in,
  double scale_linear,
  double scale_angular)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = in.linear.x * scale_linear;
  out.linear.y = in.linear.y * scale_linear;
  out.linear.z = in.linear.z * scale_linear;
  out.angular.x = in.angular.x * scale_angular;
  out.angular.y = in.angular.y * scale_angular;
  out.angular.z = in.angular.z * scale_angular;
  return out;
}

inline geometry_msgs::msg::Twist clamp_twist(
  const geometry_msgs::msg::Twist & in,
  double max_linear_abs,
  double max_angular_abs)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = clamp(in.linear.x, -max_linear_abs, max_linear_abs);
  out.linear.y = clamp(in.linear.y, -max_linear_abs, max_linear_abs);
  out.linear.z = clamp(in.linear.z, -max_linear_abs, max_linear_abs);
  out.angular.x = clamp(in.angular.x, -max_angular_abs, max_angular_abs);
  out.angular.y = clamp(in.angular.y, -max_angular_abs, max_angular_abs);
  out.angular.z = clamp(in.angular.z, -max_angular_abs, max_angular_abs);
  return out;
}

inline geometry_msgs::msg::Twist zero_twist()
{
  return geometry_msgs::msg::Twist{};
}

}  // namespace amr_safety_core

#endif  // AMR_SAFETY_CORE__SAFETY_POLICY_HPP_
