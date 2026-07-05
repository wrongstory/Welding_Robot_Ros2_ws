#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "amr_motion_control_2wd/motion_common.hpp"

#include "amr_interfaces/action/amr_motion_pure_pursuit.hpp"

namespace amr_motion_control_2wd
{

class PurePursuitActionServer
{
public:
  using PurePursuit = amr_interfaces::action::AMRMotionPurePursuit;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<PurePursuit>;

  struct Waypoint
  {
    double x;
    double y;
  };

  struct ClosestPointResult
  {
    size_t segment_idx;
    double t;
    double x;
    double y;
    double cross_track_err;
    double arc_length;
  };

  struct LookaheadResult
  {
    double x;
    double y;
    double distance;
    bool   valid;
  };

  explicit PurePursuitActionServer(rclcpp::Node::SharedPtr node);

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<PurePursuit>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr       path_viz_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // /robot_pose 구독 + 신선도 판정 (기존 TF map->base_link lookup 대체)
  PoseCache pose_cache_;

  // ── Path state ──
  std::mutex            path_mutex_;
  std::vector<Waypoint> waypoints_;
  std::vector<double>   cumulative_dist_;
  double                total_path_length_{0.0};
  std::atomic<bool>     path_received_{false};

  // ── Control parameters ──
  double      ctrl_freq_hz_;
  double      default_lookahead_distance_;
  double      min_lookahead_distance_;
  double      max_lookahead_distance_;
  double      default_goal_tolerance_;
  double      max_omega_;
  double      max_timeout_sec_;
  double      lateral_abort_dist_;
  double      min_vx_;
  double      wheel_separation_;
  std::string robot_base_frame_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string action_name_;

  // ── Action callbacks ──
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PurePursuit::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  bool buildPathLocked(const nav_msgs::msg::Path & path_msg);

  bool lookupPose(double & x, double & y, double & yaw);
  void publishCmdVel(double vx, double omega);
  void publishPathMarker();
  void clearPathMarker();
};

}  // namespace amr_motion_control_2wd
