#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

namespace amr_motion_control_2wd
{

class LocalizationWatchdog
{
public:
  struct Config {
    double timeout_sec = 2.0;
    double fixed_jump_threshold = 0.3;
    double velocity_margin = 1.3;
  };

  LocalizationWatchdog(Config config, rclcpp::Logger logger);

  // Callback thread에서 호출
  void updatePose(double x, double y, double yaw);

  // Execute thread에서 호출
  void setCurrentSpeed(double speed_mps);
  bool checkHealth();
  void reset();  // action 시작 시 초기화

  // Getters
  double x() const { return loc_x_.load(); }
  double y() const { return loc_y_.load(); }
  double yaw() const { return loc_yaw_.load(); }
  bool poseReceived() const { return pose_received_.load(); }

private:
  std::mutex mutex_;  // last_pose_time_ 보호
  std::chrono::steady_clock::time_point last_pose_time_;
  std::atomic<bool> pose_received_{false};
  std::atomic<bool> position_jump_detected_{false};
  std::atomic<double> loc_x_{0.0};
  std::atomic<double> loc_y_{0.0};
  std::atomic<double> loc_yaw_{0.0};
  std::atomic<double> max_cmd_speed_{0.0};
  double prev_x_{0.0};
  double prev_y_{0.0};
  std::atomic<bool> initialized_{false};
  Config config_;
  rclcpp::Logger logger_;
};

}  // namespace amr_motion_control_2wd
