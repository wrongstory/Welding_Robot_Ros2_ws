#ifndef AMR_MOTION_CONTROL_2WD__TURN_ACTION_SERVER_HPP_
#define AMR_MOTION_CONTROL_2WD__TURN_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "amr_interfaces/action/amr_motion_turn.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_safety_core/safety_subscriber.hpp"

namespace amr_motion_control_2wd
{

class TurnActionServer
{
public:
  using Turn = amr_interfaces::action::AMRMotionTurn;
  using GoalHandleTurn = rclcpp_action::ServerGoalHandle<Turn>;

  explicit TurnActionServer(rclcpp::Node::SharedPtr node);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Turn::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTurn> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleTurn> goal_handle);

  void execute(const std::shared_ptr<GoalHandleTurn> goal_handle);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Turn>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<amr_safety_core::SafetySubscriber> safety_sub_;

  // IMU feedback state (atomic: execute thread + callback thread)
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<bool> imu_received_{false};

  // Control parameters (from YAML)
  double control_rate_hz_{50.0};

  // Turn precision parameters (shared with Spin, via safe_param)
  double imu_deadband_rad_{0.001};
  double min_speed_dps_{2.0};
  double fine_correction_threshold_deg_{0.3};
  double fine_correction_speed_dps_{3.0};
  double fine_correction_timeout_sec_{3.0};
  int    settling_delay_ms_{200};
};

}  // namespace amr_motion_control_2wd

#endif  // AMR_MOTION_CONTROL_2WD__TURN_ACTION_SERVER_HPP_
