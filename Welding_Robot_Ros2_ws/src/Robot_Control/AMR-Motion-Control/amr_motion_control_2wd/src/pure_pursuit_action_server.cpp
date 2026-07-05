#include "amr_motion_control_2wd/pure_pursuit_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{

// ─── Constructor ──────────────────────────────────────────────────────────────
PurePursuitActionServer::PurePursuitActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  pose_cache_(node_.get())
{
  ctrl_freq_hz_               = safeParam(node_, "pure_pursuit_ctrl_freq_hz",              50.0);
  default_lookahead_distance_ = safeParam(node_, "pure_pursuit_default_lookahead_distance", 0.5);
  min_lookahead_distance_     = safeParam(node_, "pure_pursuit_min_lookahead_distance",     0.2);
  max_lookahead_distance_     = safeParam(node_, "pure_pursuit_max_lookahead_distance",     2.0);
  default_goal_tolerance_     = safeParam(node_, "pure_pursuit_default_goal_tolerance",     0.1);
  max_omega_                  = safeParam(node_, "pure_pursuit_max_omega",                  1.5);
  max_timeout_sec_            = safeParam(node_, "pure_pursuit_max_timeout_sec",           120.0);
  lateral_abort_dist_         = safeParam(node_, "pure_pursuit_lateral_abort_dist",         1.5);
  min_vx_                     = safeParam(node_, "pure_pursuit_min_vx",                     0.02);
  wheel_separation_           = safeParam(node_, "wheel_separation",                        0.48);
  robot_base_frame_           = safeParam(node_, "pure_pursuit_robot_base_frame",
                                          std::string("base_link"));
  path_topic_                 = safeParam(node_, "pure_pursuit_path_topic",
                                          std::string("pure_pursuit/path"));
  cmd_vel_topic_              = safeParam(node_, "pure_pursuit_cmd_vel_topic",
                                          std::string("cmd_vel"));
  action_name_                = safeParam(node_, "pure_pursuit_action_name",
                                          std::string("amr_motion_pure_pursuit"));

  cmd_vel_pub_  = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "pure_pursuit/path_viz", rclcpp::QoS(10).transient_local());

  auto path_qos = rclcpp::QoS(10).reliable();  // VOLATILE — matches goal_pose_bridge publisher
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, path_qos,
    std::bind(&PurePursuitActionServer::pathCallback, this, _1));

  action_server_ = rclcpp_action::create_server<PurePursuit>(
    node_,
    action_name_,
    std::bind(&PurePursuitActionServer::handleGoal,     this, _1, _2),
    std::bind(&PurePursuitActionServer::handleCancel,   this, _1),
    std::bind(&PurePursuitActionServer::handleAccepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
    "PurePursuitActionServer ready: action=%s path_topic=%s cmd_vel=%s "
    "base_frame=%s L=%.3f freq=%.0fHz",
    action_name_.c_str(), path_topic_.c_str(), cmd_vel_topic_.c_str(),
    robot_base_frame_.c_str(), wheel_separation_, ctrl_freq_hz_);
}

// ─── Path subscription ───────────────────────────────────────────────────────
void PurePursuitActionServer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (buildPathLocked(*msg)) {
    path_received_.store(true);
    RCLCPP_INFO(node_->get_logger(),
      "PurePursuitActionServer: path received (%zu wpts, %.3f m)",
      waypoints_.size(), total_path_length_);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: received invalid path (%zu poses)",
      msg->poses.size());
  }
}

bool PurePursuitActionServer::buildPathLocked(const nav_msgs::msg::Path & path_msg)
{
  waypoints_.clear();
  cumulative_dist_.clear();
  total_path_length_ = 0.0;

  if (path_msg.poses.size() < 2) {
    return false;
  }

  waypoints_.reserve(path_msg.poses.size());
  for (const auto & ps : path_msg.poses) {
    waypoints_.push_back({ps.pose.position.x, ps.pose.position.y});
  }

  cumulative_dist_.resize(waypoints_.size(), 0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    double dx = waypoints_[i].x - waypoints_[i - 1].x;
    double dy = waypoints_[i].y - waypoints_[i - 1].y;
    cumulative_dist_[i] = cumulative_dist_[i - 1] + std::hypot(dx, dy);
  }

  total_path_length_ = cumulative_dist_.back();
  return total_path_length_ >= 1e-4;
}

// ─── Action callbacks ─────────────────────────────────────────────────────────
rclcpp_action::GoalResponse PurePursuitActionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PurePursuit::Goal> goal)
{
  if (goal->max_linear_speed <= 0.0 || goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: invalid params (speed=%.3f accel=%.3f)",
      goal->max_linear_speed, goal->acceleration);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!path_received_.load()) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: no path received yet on %s, rejecting",
      path_topic_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_->get_logger(),
    "PurePursuitActionServer: goal accepted (v=%.3f a=%.3f ld=%.3f tol=%.3f)",
    goal->max_linear_speed, goal->acceleration,
    goal->lookahead_distance, goal->goal_tolerance);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PurePursuitActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "PurePursuitActionServer: cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PurePursuitActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PurePursuitActionServer::execute, this, goal_handle)}.detach();
}

// ─── Main execution loop ──────────────────────────────────────────────────────
void PurePursuitActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  ActiveAction expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::PURE_PURSUIT)) {
    RCLCPP_WARN(node_->get_logger(),
      "PurePursuitActionServer: another action is active (%s), aborting goal",
      to_string(g_active_action.load()));
    auto result = std::make_shared<PurePursuit::Result>();
    result->status = -2;
    try { goal_handle->abort(result); } catch (...) {}
    return;
  }
  ActionGuard action_guard;

  const auto & goal       = goal_handle->get_goal();
  const auto   start_time = node_->now();

  // Snapshot path waypoints under lock
  std::vector<Waypoint> local_wpts;
  std::vector<double>   local_cum;
  double                local_total_len = 0.0;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (waypoints_.size() < 2) {
      RCLCPP_WARN(node_->get_logger(),
        "PurePursuitActionServer: no valid path at execute");
      auto result = std::make_shared<PurePursuit::Result>();
      result->status       = -2;
      result->elapsed_time = 0.0;
      try { goal_handle->abort(result); } catch (...) {}
      return;
    }
    local_wpts      = waypoints_;
    local_cum       = cumulative_dist_;
    local_total_len = total_path_length_;
  }

  double lookahead_dist = (goal->lookahead_distance > 1e-6)
    ? goal->lookahead_distance
    : default_lookahead_distance_;
  lookahead_dist = std::clamp(lookahead_dist,
                              min_lookahead_distance_,
                              max_lookahead_distance_);

  double goal_tolerance = (goal->goal_tolerance > 1e-6)
    ? goal->goal_tolerance
    : default_goal_tolerance_;

  amr_motion_control::TrapezoidalProfile profile(
    local_total_len,
    goal->max_linear_speed,
    goal->acceleration,
    0.0);

  publishPathMarker();

  rclcpp::Rate rate(ctrl_freq_hz_);

  double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;

  // Wait for first TF pose (5s)
  {
    auto pose_wait_deadline = node_->now() + rclcpp::Duration::from_seconds(5.0);
    while (rclcpp::ok() && !lookupPose(rob_x, rob_y, rob_yaw)) {
      if (node_->now() > pose_wait_deadline) {
        RCLCPP_ERROR(node_->get_logger(),
          "PurePursuitActionServer: no TF pose in 5 s, aborting");
        publishCmdVel(0.0, 0.0);
        auto result = std::make_shared<PurePursuit::Result>();
        result->status       = -3;
        result->elapsed_time = (node_->now() - start_time).seconds();
        try { goal_handle->abort(result); } catch (...) {}
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "PurePursuit start: pose=(%.3f,%.3f,%.3f) path_len=%.3f wpts=%zu ld=%.3f tol=%.3f",
    rob_x, rob_y, rob_yaw, local_total_len, local_wpts.size(),
    lookahead_dist, goal_tolerance);

  double arc_length_traveled = 0.0;

  // Path snapshot is local; the subscription may overwrite member waypoints_
  // during execution, so geometry helpers below close over local vectors only.
  auto closestLocal = [&](double rx, double ry) -> ClosestPointResult {
    ClosestPointResult best{};
    best.segment_idx     = 0;
    best.t               = 0.0;
    best.x               = local_wpts[0].x;
    best.y               = local_wpts[0].y;
    best.cross_track_err = 0.0;
    best.arc_length      = 0.0;

    double min_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < local_wpts.size(); ++i) {
      const Waypoint & A = local_wpts[i];
      const Waypoint & B = local_wpts[i + 1];
      double seg_dx = B.x - A.x;
      double seg_dy = B.y - A.y;
      double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
      double t = 0.0;
      if (seg_len_sq > 1e-12) {
        t = ((rx - A.x) * seg_dx + (ry - A.y) * seg_dy) / seg_len_sq;
        t = std::clamp(t, 0.0, 1.0);
      }
      double cx = A.x + t * seg_dx;
      double cy = A.y + t * seg_dy;
      double dist_sq = (rx - cx) * (rx - cx) + (ry - cy) * (ry - cy);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        best.segment_idx = i;
        best.t = t;
        best.x = cx;
        best.y = cy;
        double seg_len = std::sqrt(seg_len_sq);
        double ux = (seg_len > 1e-12) ? seg_dx / seg_len : 1.0;
        double uy = (seg_len > 1e-12) ? seg_dy / seg_len : 0.0;
        best.cross_track_err = ux * (ry - cy) - uy * (rx - cx);
        best.arc_length = local_cum[i] + t * seg_len;
      }
    }
    return best;
  };

  auto lookaheadLocal = [&](const ClosestPointResult & closest,
                            double ld) -> LookaheadResult {
    double target_arc = closest.arc_length + ld;
    if (target_arc >= local_total_len) {
      const Waypoint & last = local_wpts.back();
      double dx = last.x - closest.x;
      double dy = last.y - closest.y;
      return {last.x, last.y, std::hypot(dx, dy), true};
    }
    size_t seg = closest.segment_idx;
    for (size_t i = seg; i + 1 < local_wpts.size(); ++i) {
      if (local_cum[i] <= target_arc && target_arc <= local_cum[i + 1]) {
        seg = i;
        break;
      }
      if (i + 2 == local_wpts.size()) {
        seg = i;
      }
    }
    double seg_len = local_cum[seg + 1] - local_cum[seg];
    double t = 0.0;
    if (seg_len > 1e-12) {
      t = (target_arc - local_cum[seg]) / seg_len;
      t = std::clamp(t, 0.0, 1.0);
    }
    const Waypoint & A = local_wpts[seg];
    const Waypoint & B = local_wpts[seg + 1];
    double lx = A.x + t * (B.x - A.x);
    double ly = A.y + t * (B.y - A.y);
    double dx = lx - closest.x;
    double dy = ly - closest.y;
    return {lx, ly, std::hypot(dx, dy), true};
  };

  double L_actual = std::max(wheel_separation_, 0.10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                  = -1;
      result->actual_distance         = arc_length_traveled;
      result->final_cross_track_error = 0.0;
      result->elapsed_time            = (node_->now() - start_time).seconds();
      try { goal_handle->canceled(result); } catch (...) {}
      return;
    }

    const double elapsed = (node_->now() - start_time).seconds();
    if (elapsed > max_timeout_sec_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      RCLCPP_ERROR(node_->get_logger(),
        "PurePursuitActionServer: timeout %.1f s", elapsed);
      auto result = std::make_shared<PurePursuit::Result>();
      result->status       = -3;
      result->elapsed_time = elapsed;
      try { goal_handle->abort(result); } catch (...) {}
      return;
    }

    if (!lookupPose(rob_x, rob_y, rob_yaw)) {
      rate.sleep();
      continue;
    }

    const auto closest = closestLocal(rob_x, rob_y);
    arc_length_traveled = closest.arc_length;
    const double cte    = closest.cross_track_err;

    if (std::abs(cte) > lateral_abort_dist_) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                  = -3;
      result->actual_distance         = arc_length_traveled;
      result->final_cross_track_error = cte;
      result->elapsed_time            = elapsed;
      try { goal_handle->abort(result); } catch (...) {}
      RCLCPP_ERROR(node_->get_logger(),
        "PurePursuitActionServer: lateral abort cte=%.3f", cte);
      return;
    }

    const Waypoint & last_wp = local_wpts.back();
    const double dist_to_goal = std::hypot(last_wp.x - rob_x, last_wp.y - rob_y);

    if (dist_to_goal < goal_tolerance) {
      publishCmdVel(0.0, 0.0);
      clearPathMarker();
      auto result = std::make_shared<PurePursuit::Result>();
      result->status                  = 0;
      result->actual_distance         = arc_length_traveled;
      result->final_cross_track_error = cte;
      result->elapsed_time            = (node_->now() - start_time).seconds();
      try { goal_handle->succeed(result); } catch (...) {}
      RCLCPP_INFO(node_->get_logger(),
        "PurePursuitActionServer: arrived (dist=%.3f cte=%.3f t=%.2fs)",
        arc_length_traveled, cte, result->elapsed_time);
      return;
    }

    const auto profile_out = profile.getSpeed(
      std::min(arc_length_traveled, local_total_len));
    double vx = profile_out.speed;
    if (profile_out.phase != amr_motion_control::ProfilePhase::DONE &&
        vx < min_vx_ && dist_to_goal > goal_tolerance) {
      vx = min_vx_;
    }

    const auto lookahead = lookaheadLocal(closest, lookahead_dist);

    double alpha = std::atan2(lookahead.y - rob_y, lookahead.x - rob_x) - rob_yaw;
    alpha = normalizeAngle(alpha);

    double omega = 2.0 * vx * std::sin(alpha) / L_actual;
    omega = std::clamp(omega, -max_omega_, max_omega_);

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "PurePursuit: rob=(%.2f,%.2f,%.1fdeg) arc=%.2f/%.2f cte=%.3f vx=%.2f w=%.2f a=%.1fdeg",
      rob_x, rob_y, rob_yaw * 180.0 / M_PI,
      arc_length_traveled, local_total_len, cte,
      vx, omega, alpha * 180.0 / M_PI);

    publishCmdVel(vx, omega);

    {
      auto feedback = std::make_shared<PurePursuit::Feedback>();
      feedback->current_distance       = arc_length_traveled;
      feedback->remaining_distance     = std::max(0.0, local_total_len - arc_length_traveled);
      feedback->cross_track_error      = cte;
      feedback->heading_error          = alpha * 180.0 / M_PI;
      feedback->current_speed          = vx;
      feedback->current_waypoint_index = static_cast<uint32_t>(closest.segment_idx);
      feedback->total_waypoints        = static_cast<uint32_t>(local_wpts.size());
      feedback->phase                  = 2;
      try { goal_handle->publish_feedback(feedback); } catch (...) {}
    }

    rate.sleep();
  }

  publishCmdVel(0.0, 0.0);
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
bool PurePursuitActionServer::lookupPose(double & x, double & y, double & yaw)
{
  if (pose_cache_.getPose(x, y, yaw)) {
    return true;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "PurePursuit /robot_pose stale or not received");
  return false;
}

void PurePursuitActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

void PurePursuitActionServer::publishPathMarker()
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp    = node_->now();
  std::lock_guard<std::mutex> lock(path_mutex_);
  path_msg.poses.reserve(waypoints_.size());
  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x    = wp.x;
    ps.pose.position.y    = wp.y;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }
  path_viz_pub_->publish(path_msg);
}

void PurePursuitActionServer::clearPathMarker()
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  empty.header.stamp    = node_->now();
  path_viz_pub_->publish(empty);
}

}  // namespace amr_motion_control_2wd
