#ifndef AMR_MOTION_CONTROL_2WD__SPIN_ACTION_SERVER_HPP_
#define AMR_MOTION_CONTROL_2WD__SPIN_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_safety_core/safety_subscriber.hpp"

namespace amr_motion_control_2wd
{

class SpinActionServer
{
public:
  using Spin = amr_interfaces::action::AMRMotionSpin;
  using GoalHandleSpin = rclcpp_action::ServerGoalHandle<Spin>;

  explicit SpinActionServer(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Spin::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpin> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleSpin> goal_handle);

  void execute(const std::shared_ptr<GoalHandleSpin> goal_handle);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Spin>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<amr_safety_core::SafetySubscriber> safety_sub_;

  // 로봇 pose: /robot_pose 구독 (PoseCache, 기존 TF map->base_link 대체)
  PoseCache pose_cache_;

  // IMU feedback state (atomic: execute thread + callback thread)
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool> imu_received_{false};

  // Control parameters (from YAML)
  double control_rate_hz_{50.0};

  // Spin precision parameters (from YAML)
  double imu_deadband_rad_{0.001};
  double min_speed_dps_{2.0};
  double fine_correction_threshold_deg_{0.3};
  double fine_correction_speed_dps_{3.0};
  double fine_correction_timeout_sec_{3.0};
  int    settling_delay_ms_{200};

  // Fine-correction PID gains (Phase 3.5 — replaces bang-bang constant-speed).
  // omega_dps = Kp*e + Ki*∫e + Kd*de/dt,  e = signed remaining error [deg].
  double kp_spin_{1.5};
  double ki_spin_{0.0};
  double kd_spin_{0.5};
  double integral_limit_deg_{30.0};  // anti-windup clamp on ∫e

  // Coarse 조기종료 band [deg]: coarse 를 (target - band)에서 끝내 overshoot 상쇄. 0 = 기존 동작.
  double coarse_exit_band_deg_{0.0};

  // idle keep-alive 타이머: goal 없을 때 cmd_vel=0 발행 → ESP32 st:5 유지 (재투입 불필요).
  rclcpp::TimerBase::SharedPtr idle_timer_;
};

}  // namespace amr_motion_control_2wd

#endif  // AMR_MOTION_CONTROL_2WD__SPIN_ACTION_SERVER_HPP_
