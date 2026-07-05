#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

namespace amr_safety_core
{

class LocalizationWatchdog
{
public:
  struct Config {
    double timeout_sec = 2.0;
    double fixed_jump_threshold = 0.3;
    double velocity_margin = 1.3;
  };

  LocalizationWatchdog(Config config, rclcpp::Logger logger)
  : config_(config), logger_(logger)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_time_ = std::chrono::steady_clock::now();
  }

  // Callback thread에서 호출
  void updatePose(double x, double y, double yaw)
  {
    // Position jump detection (velocity-aware)
    if (initialized_.load()) {
      double jump_dist = std::hypot(x - prev_x_, y - prev_y_);

      double dt_poses = 0.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        dt_poses = std::chrono::duration<double>(now - last_pose_time_).count();
      }
      double expected_dist = max_cmd_speed_.load() * dt_poses;
      double effective_threshold = std::max(config_.fixed_jump_threshold,
                                            expected_dist * config_.velocity_margin);

      if (jump_dist > effective_threshold) {
        position_jump_detected_.store(true);
        RCLCPP_WARN(logger_,
          "Pose jump: %.3f m (exp=%.3f, thr=%.3f) (%.2f,%.2f)->(%.2f,%.2f)",
          jump_dist, expected_dist, effective_threshold,
          prev_x_, prev_y_, x, y);
      }
    }
    prev_x_ = x;
    prev_y_ = y;
    initialized_.store(true);

    loc_x_.store(x);
    loc_y_.store(y);
    loc_yaw_.store(yaw);
    pose_received_.store(true);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_pose_time_ = std::chrono::steady_clock::now();
    }
  }

  // Execute thread에서 호출
  void setCurrentSpeed(double speed_mps)
  {
    max_cmd_speed_.store(speed_mps);
  }

  bool checkHealth()
  {
    // 1. Timeout check (steady_clock)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto now = std::chrono::steady_clock::now();
      double pose_age = std::chrono::duration<double>(now - last_pose_time_).count();
      if (pose_age > config_.timeout_sec) {
        RCLCPP_WARN(logger_,
          "Localization timeout: pose age=%.3f s > %.3f s",
          pose_age, config_.timeout_sec);
        return false;
      }
    }

    // 2. Position jump detection (atomic exchange로 TOCTOU 방지)
    if (position_jump_detected_.exchange(false)) {
      RCLCPP_WARN(logger_,
        "Position jump detected (>%.2f m)", config_.fixed_jump_threshold);
      return false;
    }

    return true;
  }

  void reset()
  {
    position_jump_detected_.store(false);
    initialized_.store(false);
    max_cmd_speed_.store(0.0);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_pose_time_ = std::chrono::steady_clock::now();
    }
  }

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

}  // namespace amr_safety_core
