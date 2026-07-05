#ifndef AMR_MOTION_CONTROL_2WD__TRANSLATE_REVERSE_ACTION_SERVER_HPP_
#define AMR_MOTION_CONTROL_2WD__TRANSLATE_REVERSE_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_translate.hpp"
#include "amr_interfaces/srv/update_translate_endpoint.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_motion_control_2wd/recursive_moving_average.hpp"
#include "amr_safety_core/localization_watchdog.hpp"
#include "amr_safety_core/safety_subscriber.hpp"

namespace amr_motion_control_2wd
{
using amr_safety_core::LocalizationWatchdog;

class TranslateReverseActionServer
{
public:
  using Translate = amr_interfaces::action::AMRMotionTranslate;
  using GoalHandleTranslate = rclcpp_action::ServerGoalHandle<Translate>;

  explicit TranslateReverseActionServer(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Translate::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTranslate> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleTranslate> goal_handle);

  void execute(const std::shared_ptr<GoalHandleTranslate> goal_handle);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Translate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;

  // Pose acquisition (latest from /robot_pose via shared PoseCache)
  PoseCache pose_cache_;

  // IMU feedback state (atomic: execute thread + callback thread)
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool> imu_received_{false};

  // Localization watchdog
  std::optional<LocalizationWatchdog> watchdog_;

  // Endpoint update service (velocity continuity)
  rclcpp::Service<amr_interfaces::srv::UpdateTranslateEndpoint>::SharedPtr update_endpoint_srv_;
  void onUpdateEndpoint(
    const std::shared_ptr<amr_interfaces::srv::UpdateTranslateEndpoint::Request> req,
    std::shared_ptr<amr_interfaces::srv::UpdateTranslateEndpoint::Response> res);
  std::atomic<bool> endpoint_update_pending_{false};
  double pending_end_x_{0.0};
  double pending_end_y_{0.0};
  std::atomic<bool> pending_has_next_{false};
  std::mutex endpoint_mutex_;

  // Safety (speed limit + status via SafetySubscriber — translate/spin/yaw 와 동일 추상)
  std::unique_ptr<amr_safety_core::SafetySubscriber> safety_sub_;

  // Reverse mode flag
  bool is_reverse_{false};

  // Inline path controller state (inlined PathController)
  double path_start_x_{0.0}, path_start_y_{0.0};
  double path_end_x_{0.0}, path_end_y_{0.0};
  double theta_path_{0.0};
  double target_distance_{0.0};
  double path_ux_{0.0}, path_uy_{0.0};  // path unit vector
  double prev_e_theta_{0.0};
  double prev_omega_{0.0};
  amr_motion_control::RecursiveMovingAverage e_theta_filter_;

  // Control parameters (from YAML)
  double control_rate_hz_{50.0};
  double min_vx_{0.02};                 // m/s
  double behind_start_speed_{0.2};      // m/s (projection<0 시 고정 접근 속도, creep 모드)
  bool   behind_start_accelerate_{false};  // true=시작점 뒤에서부터 가속(프로파일 전치), false=behind_start_speed_ 고정
  double behind_start_kick_{0.1};       // m/s accelerate 모드 출발 kick(정지마찰 극복, Issue 8 fix)
  double goal_reach_threshold_{0.05};   // m
  double arrival_tolerance_{0.03};     // m (도착 판정 허용 오차)
  double max_timeout_sec_{60.0};        // sec
  double localization_timeout_sec_{2.0};
  double position_jump_threshold_{0.3}; // m
  bool enable_localization_watchdog_{true};
  double walk_accel_limit_{0.5};        // m/s²
  double walk_decel_limit_{1.0};        // m/s²

  // 도착 2단 감속 캡 (T-AMR rosbag 측정 기반, 완전정지 jerk 저감)
  //  1단(원거리) v_cap=approach_gain_·remaining / 2단(근거리,v<stop_speed_) v_cap=sqrt(2·stop_decel_·rem).
  //  전환 잔여거리 = stop_speed_^2/(2·stop_decel_). 도착 ramp-to-0 도 stop_decel_ 로 완만.
  double approach_gain_{1.5};           // 1/s  (1단 거리비례 게인)
  double stop_speed_{0.2};              // m/s  (1단↔2단 전환 속도)
  double stop_decel_{0.1};              // m/s² (2단 정속감속 + 최종 ramp 감속도)
  // mpc_light 감속 (jerk-제한 S-curve v_cap + 출력 rate-limit)
  double jerk_max_{1.0};                // m/s³
  double a_decel_max_{0.4};             // m/s²
  double should_stop_speed_{0.1};       // m/s

  // Inline PathController parameters
  double Kp_heading_{1.0};
  double Kd_heading_{0.3};
  double K_stanley_{2.0};
  double K_soft_{1.0};
  double max_omega_{1.0};              // rad/s
  double alpha_max_{0.5};             // rad/s² (omega angular acceleration limit)
  double heading_threshold_{0.785};   // rad (45 deg) for initial validation
  double max_lateral_offset_{1.0};    // m for initial validation
  int heading_filter_window_{5};
};

}  // namespace amr_motion_control_2wd

#endif  // AMR_MOTION_CONTROL_2WD__TRANSLATE_REVERSE_ACTION_SERVER_HPP_
