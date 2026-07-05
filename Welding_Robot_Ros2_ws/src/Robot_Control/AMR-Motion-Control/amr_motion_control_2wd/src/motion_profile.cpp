#include "amr_motion_control_2wd/motion_profile.hpp"

#include <algorithm>

namespace amr_motion_control
{

TrapezoidalProfile::TrapezoidalProfile(
  double target_distance,
  double max_speed,
  double acceleration,
  double exit_speed)
: target_distance_(target_distance),
  max_speed_(max_speed),
  acceleration_(acceleration),
  exit_speed_(exit_speed)
{
  // Distance needed to accelerate from 0 to max_speed:
  //   d_accel = v_max² / (2 * a)
  double d_accel_full = (max_speed_ * max_speed_) / (2.0 * acceleration_);

  // Distance needed to decelerate from max_speed to exit_speed:
  //   d_decel = (v_max² - v_exit²) / (2 * a)
  double d_decel_full = (max_speed_ * max_speed_ - exit_speed_ * exit_speed_) / (2.0 * acceleration_);

  if (d_accel_full + d_decel_full <= target_distance_) {
    // Trapezoidal: enough room for full accel + cruise + decel
    is_triangular_ = false;
    peak_speed_ = max_speed_;
    accel_dist_ = d_accel_full;
    decel_start_ = target_distance_ - d_decel_full;
  } else {
    // Triangular: not enough room for full speed
    // Peak speed from energy balance: v_peak² / (2a) + (v_peak² - v_exit²) / (2a) = target_distance
    // => v_peak² = a * target_distance + v_exit² / 2
    is_triangular_ = true;
    peak_speed_ = std::sqrt(acceleration_ * target_distance_ + exit_speed_ * exit_speed_ / 2.0);
    // Clamp: peak_speed must be at least exit_speed
    peak_speed_ = std::max(peak_speed_, exit_speed_);
    accel_dist_ = (peak_speed_ * peak_speed_) / (2.0 * acceleration_);
    decel_start_ = accel_dist_;  // No cruise phase
  }
}

ProfileOutput TrapezoidalProfile::getSpeed(double current_position) const
{
  ProfileOutput out{};

  // Clamp position
  double pos = std::max(0.0, std::min(current_position, target_distance_));

  if (pos >= target_distance_) {
    out.speed = exit_speed_;
    out.phase = ProfilePhase::DONE;
    return out;
  }

  if (pos < accel_dist_) {
    // Acceleration phase: v = sqrt(2 * a * pos)
    out.speed = std::sqrt(2.0 * acceleration_ * pos);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::ACCEL;
  } else if (pos < decel_start_) {
    // Cruise phase (trapezoidal only): v = peak_speed
    out.speed = peak_speed_;
    out.phase = ProfilePhase::CRUISE;
  } else {
    // Deceleration phase: v = sqrt(v_exit² + 2 * a * remaining)
    // As remaining -> 0, speed -> exit_speed_
    double remaining = target_distance_ - pos;
    out.speed = std::sqrt(exit_speed_ * exit_speed_ + 2.0 * acceleration_ * remaining);
    out.speed = std::min(out.speed, peak_speed_);
    out.phase = ProfilePhase::DECEL;
  }

  return out;
}

bool TrapezoidalProfile::isComplete(double current_position) const
{
  return current_position >= target_distance_;
}

}  // namespace amr_motion_control
