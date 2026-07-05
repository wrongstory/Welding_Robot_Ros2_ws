#ifndef AMR_MOTION_CONTROL__MOTION_PROFILE_HPP_
#define AMR_MOTION_CONTROL__MOTION_PROFILE_HPP_

#include <cmath>
#include <cstdint>

namespace amr_motion_control
{

/// Phase within the trapezoidal profile
enum class ProfilePhase : uint8_t
{
  ACCEL  = 1,  // Acceleration phase
  CRUISE = 2,  // Constant speed phase
  DECEL  = 3,  // Deceleration phase
  DONE   = 4   // Profile complete
};

/// Output of getSpeed() query
struct ProfileOutput
{
  double speed;         // Current speed (always >= 0, unit matches input)
  ProfilePhase phase;   // Current phase
};

/**
 * TrapezoidalProfile — Symmetric trapezoidal/triangular motion profile
 *
 * Reference: Implementation Plan §5.4.3
 *
 * Automatically selects trapezoidal or triangular profile:
 * - Trapezoidal: distance >= 2 * accel_distance  (accel + cruise + decel)
 * - Triangular:  distance <  2 * accel_distance   (accel + decel, reduced peak)
 *
 * Usage:
 *   TrapezoidalProfile profile(target, max_speed, acceleration);
 *   while (!profile.isComplete(current_pos)) {
 *     auto out = profile.getSpeed(current_pos);
 *     // use out.speed and out.phase
 *   }
 *
 * Units are generic: works for both angular (deg, deg/s, deg/s²)
 * and linear (m, m/s, m/s²) quantities.
 */
class TrapezoidalProfile
{
public:
  /**
   * @param target_distance  Total distance to travel (always > 0)
   * @param max_speed        Maximum speed (always > 0)
   * @param acceleration     Acceleration magnitude (always > 0)
   * @param exit_speed       Speed at the end of the profile (default 0.0)
   */
  TrapezoidalProfile(
    double target_distance,
    double max_speed,
    double acceleration,
    double exit_speed = 0.0);

  /**
   * Get speed at a given position along the profile.
   *
   * @param current_position  Distance traveled so far (>= 0)
   * @return ProfileOutput with speed and current phase
   */
  ProfileOutput getSpeed(double current_position) const;

  /**
   * Check if the profile is complete.
   *
   * @param current_position  Distance traveled so far
   * @return true if current_position >= target_distance
   */
  bool isComplete(double current_position) const;

  /// Get the peak speed (may be < max_speed for triangular profiles)
  double peakSpeed() const { return peak_speed_; }

  /// Get the acceleration distance
  double accelDistance() const { return accel_dist_; }

  /// Get the deceleration start position
  double decelStart() const { return decel_start_; }

  /// Get the total target distance
  double targetDistance() const { return target_distance_; }

  /// Get the exit speed
  double exitSpeed() const { return exit_speed_; }

private:
  double target_distance_;
  double max_speed_;
  double acceleration_;
  double exit_speed_;

  // Computed in constructor
  double peak_speed_;    // Actual peak speed (may be < max_speed for triangular)
  double accel_dist_;    // Distance to accelerate from 0 to peak_speed
  double decel_start_;   // Position where deceleration begins
  bool   is_triangular_; // True if profile is triangular (no cruise phase)
};

}  // namespace amr_motion_control

#endif  // AMR_MOTION_CONTROL__MOTION_PROFILE_HPP_
