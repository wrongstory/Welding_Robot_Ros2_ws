#ifndef AMR_MOTION_CONTROL_2WD__YAW_CONTROL_ACTION_SERVER_HPP_
#define AMR_MOTION_CONTROL_2WD__YAW_CONTROL_ACTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_yaw_control.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_safety_core/localization_watchdog.hpp"
#include "amr_safety_core/safety_subscriber.hpp"

namespace amr_motion_control_2wd
{
using amr_safety_core::LocalizationWatchdog;

class YawControlActionServer
{
public:
  using YawControl = amr_interfaces::action::AMRMotionYawControl;
  using GoalHandleYawControl = rclcpp_action::ServerGoalHandle<YawControl>;

  explicit YawControlActionServer(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const YawControl::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleYawControl> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleYawControl> goal_handle);

  void execute(const std::shared_ptr<GoalHandleYawControl> goal_handle);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<YawControl>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Pose cache (/robot_pose, map->base_link via tr_works_pose_publisher)
  PoseCache pose_cache_;

  // IMU feedback state (atomic: execute thread + callback thread)
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool> imu_received_{false};

  // Localization watchdog
  std::optional<LocalizationWatchdog> watchdog_;

  // Safety subscriber
  std::unique_ptr<amr_safety_core::SafetySubscriber> safety_sub_;

  // Control parameters (from YAML)
  double control_rate_hz_{20.0};
  double min_vx_{0.02};                 // m/s
  double goal_reach_threshold_{0.05};   // m
  double max_timeout_sec_{60.0};        // sec
  double localization_timeout_sec_{2.0};
  double position_jump_threshold_{0.3}; // m
  bool enable_localization_watchdog_{true};
  double walk_accel_limit_{0.5};        // m/s^2
  double walk_decel_limit_{1.0};        // m/s^2

  // Inlined PathController parameters
  double Kp_heading_{1.0};
  double Kd_heading_{0.3};
  double max_omega_{1.0};               // rad/s
  double heading_threshold_{3.14159};   // rad (180 deg default)
  double max_lateral_offset_{5.0};      // m
  double alpha_max_{0.5};               // rad/s^2
  double min_turning_radius_{0.7};      // m
  int    heading_filter_window_{5};

  // Inlined TransientGuard parameter
  double omega_rate_limit_{0.5};        // rad/s per cycle
};

}  // namespace amr_motion_control_2wd

#endif  // AMR_MOTION_CONTROL_2WD__YAW_CONTROL_ACTION_SERVER_HPP_
