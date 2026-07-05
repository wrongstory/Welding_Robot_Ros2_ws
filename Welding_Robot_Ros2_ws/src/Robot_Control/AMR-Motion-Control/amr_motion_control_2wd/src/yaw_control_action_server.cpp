#include "amr_motion_control_2wd/yaw_control_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_safety_core/abort_codes.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace amr_motion_control_2wd
{

using namespace std::placeholders;

// ════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════

YawControlActionServer::YawControlActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  pose_cache_(node.get())
{
  // ── Common control parameters ──
  control_rate_hz_ = safeParam(node_, "control_rate_hz", 20.0);

  // ── YawControl-specific parameters (omega-only, vy=0) ──
  Kp_heading_ = safeParam(node_, "yaw_control_Kp_heading", 1.0);
  Kd_heading_ = safeParam(node_, "yaw_control_Kd_heading", 0.3);
  max_omega_ = safeParam(node_, "yaw_control_max_omega", 1.0);
  double heading_threshold_deg = safeParam(node_, "yaw_control_heading_threshold_deg", 180.0);
  heading_threshold_ = heading_threshold_deg * M_PI / 180.0;
  max_lateral_offset_ = safeParam(node_, "yaw_control_max_lateral_offset", 5.0);
  alpha_max_ = safeParam(node_, "yaw_control_alpha_max", 0.5);
  min_turning_radius_ = safeParam(node_, "yaw_control_min_turning_radius", 0.7);
  heading_filter_window_ = safeParam(node_, "yaw_control_heading_filter_window", 5);
  goal_reach_threshold_ = safeParam(node_, "yaw_control_goal_reach_threshold", 0.05);
  min_vx_ = safeParam(node_, "yaw_control_min_vx", 0.02);
  max_timeout_sec_ = safeParam(node_, "yaw_control_max_timeout_sec", 60.0);
  localization_timeout_sec_ = safeParam(node_, "yaw_control_localization_timeout_sec", 2.0);
  position_jump_threshold_ = safeParam(node_, "yaw_control_position_jump_threshold", 0.3);
  enable_localization_watchdog_ = safeParam(node_, "yaw_control_enable_localization_watchdog", true);

  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec = localization_timeout_sec_;
  wd_cfg.fixed_jump_threshold = position_jump_threshold_;
  wd_cfg.velocity_margin = 1.3;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  walk_accel_limit_ = safeParam(node_, "yaw_control_walk_accel_limit", 0.5);
  walk_decel_limit_ = safeParam(node_, "yaw_control_walk_decel_limit", 1.0);
  omega_rate_limit_ = safeParam(node_, "transient_omega_rate_limit", 0.5);

  // ── Publisher / Subscribers ──
  // 위치(projection)·CTE 는 PoseCache(/robot_pose, map-frame), heading 은 imu/data 고속 yaw
  //   (시작 시 1회 IMU↔map 오프셋 융합 — fuseImuYawToMap). watchdog 갱신은 execute 루프의
  //   lookupRobotPose 단일 경로로 통일하여 별도 pose_sub_(이중 /robot_pose 구독) 제거.
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "yaw_control_path", rclcpp::QoS(10).transient_local());
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::imuCallback, this, _1));

  // ── SafetySubscriber ──
  {
    amr_safety_core::SafetySubscriberConfig ss_cfg;
    safety_sub_ = std::make_unique<amr_safety_core::SafetySubscriber>(node_.get(), ss_cfg);
  }

  // ── Action server ──
  action_server_ = rclcpp_action::create_server<YawControl>(
    node_, "amr_yaw_control_action",
    std::bind(&YawControlActionServer::handle_goal, this, _1, _2),
    std::bind(&YawControlActionServer::handle_cancel, this, _1),
    std::bind(&YawControlActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer (2WD) initialized (pos=/robot_pose, heading=imu/data fused, loc_watchdog=%s)",
    enable_localization_watchdog_ ? "ON" : "OFF");
}

// ════════════════════════════════════════════════════════
//  Action callbacks
// ════════════════════════════════════════════════════════

rclcpp_action::GoalResponse YawControlActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const YawControl::Goal> goal)
{
  // Validate: start != end
  double dx = goal->end_x - goal->start_x;
  double dy = goal->end_y - goal->start_y;
  double dist = std::hypot(dx, dy);
  if (dist < 1e-6) {
    RCLCPP_WARN(node_->get_logger(), "YawControl rejected: start == end");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Validate: speed > 0, accel > 0
  if (goal->max_linear_speed <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "YawControl rejected: max_linear_speed <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "YawControl rejected: acceleration <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Mutual exclusion (전 서버 공유)
  auto expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::YAW_CONTROL)) {
    RCLCPP_WARN(node_->get_logger(), "YawControl rejected: %s is running", to_string(expected));
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node_->get_logger(),
    "YawControl goal accepted: (%.2f,%.2f)->(%.2f,%.2f), dist=%.3f m, v_max=%.2f m/s",
    goal->start_x, goal->start_y, goal->end_x, goal->end_y,
    dist, goal->max_linear_speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YawControlActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleYawControl> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "YawControl cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void YawControlActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleYawControl> goal_handle)
{
  try {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  } catch (const std::system_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn execute thread: %s", e.what());
    g_active_action.store(ActiveAction::NONE);
    auto result = std::make_shared<YawControl::Result>();
    result->status = -5;
    goal_handle->abort(result);
  }
}

// ════════════════════════════════════════════════════════
//  Main execute loop (omega-only heading control, vy=0)
// ════════════════════════════════════════════════════════

void YawControlActionServer::execute(
  const std::shared_ptr<GoalHandleYawControl> goal_handle)
{
  ActionGuard guard;  // 소멸자가 자동으로 g_active_action = NONE
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<YawControl::Feedback>();
  auto result = std::make_shared<YawControl::Result>();

  rclcpp::Rate rate(control_rate_hz_);
  const double dt = 1.0 / control_rate_hz_;

  // Helper: abort/cancel with cleanup
  auto finish_abort = [&](int8_t status, double actual_dist, double lat_err,
                          double head_err, const rclcpp::Time & start_time) {
    publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    watchdog_->setCurrentSpeed(0.0);
    // Clear path visualization
    {
      nav_msgs::msg::Path empty;
      empty.header.frame_id = "map";
      empty.header.stamp = node_->now();
      path_viz_pub_->publish(empty);
    }
    result->status = status;
    result->actual_distance = actual_dist;
    result->final_lateral_error = lat_err;
    result->final_heading_error = head_err;
    result->elapsed_time = (node_->now() - start_time).seconds();
    if (status == -1) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  };

  auto start_time = node_->now();

  // ── Inlined PathController state ──
  const double theta_path = std::atan2(
    goal->end_y - goal->start_y, goal->end_x - goal->start_x);
  const double target_distance = std::hypot(
    goal->end_x - goal->start_x, goal->end_y - goal->start_y);
  const double ux = std::cos(theta_path);
  const double uy = std::sin(theta_path);

  // Heading filter (recursive moving average)
  std::vector<double> filter_buf(heading_filter_window_, 0.0);
  int filter_idx = 0;
  double filter_sum = 0.0;
  int filter_count = 0;

  double prev_e_theta = 0.0;
  double prev_omega_pc = 0.0;  // PathController omega smoothing state

  // ── Inlined TransientGuard state ──
  double prev_omega_tg = 0.0;  // TransientGuard rate limit state

  RCLCPP_INFO(node_->get_logger(),
    "YawControl execute: path_angle=%.1f deg, target_dist=%.3f m",
    theta_path * 180.0 / M_PI, target_distance);

  // Publish path for visualization
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = node_->now();
    geometry_msgs::msg::PoseStamped ps0, ps1;
    ps0.pose.position.x = goal->start_x;
    ps0.pose.position.y = goal->start_y;
    ps0.pose.orientation.w = 1.0;
    ps1.pose.position.x = goal->end_x;
    ps1.pose.position.y = goal->end_y;
    ps1.pose.orientation.w = 1.0;
    path_msg.poses = {ps0, ps1};
    path_viz_pub_->publish(path_msg);
  }

  // ── IMU receive check ──
  // /robot_pose 수신 여부는 아래 lookupRobotPose(PoseCache) 가 직접 판정 →
  //   중복이던 watchdog_->poseReceived() 사전검사 제거.
  if (!imu_received_) {
    RCLCPP_ERROR(node_->get_logger(), "IMU data not received, aborting yaw_control");
    finish_abort(-3, 0.0, 0.0, 0.0, start_time);
    return;
  }

  // ── Initial pose from /robot_pose (map-frame, PoseCache) ──
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  if (!lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock())) {
    RCLCPP_ERROR(node_->get_logger(),
      "/robot_pose not available (stale/unreceived), aborting yaw_control");
    finish_abort(-3, 0.0, 0.0, 0.0, start_time);
    return;
  }

  // ── IMU↔map yaw 오프셋 1회 등록 (heading 고속 피드백용) ──
  // 이후 heading 오차는 fuseImuYawToMap(imu_yaw, imu_yaw_offset) - theta_path 로 계산.
  const double imu_yaw_offset = imuYawOffset(robot_yaw, last_yaw_rad_.load());

  // Inlined validateInitialPose
  {
    const double rx = robot_x - goal->start_x;
    const double ry = robot_y - goal->start_y;
    const double projection = rx * ux + ry * uy;
    const double e_d        = rx * uy - ry * ux;
    const double e_theta    = normalizeAngle(robot_yaw - theta_path);

    if (projection >= target_distance) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControl initial pose validation failed: robot already past goal (proj=%.3f)", projection);
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
    if (std::fabs(e_theta) > heading_threshold_) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControl initial pose validation failed: heading mismatch %.1f deg",
        e_theta * 180.0 / M_PI);
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
    if (std::fabs(e_d) > max_lateral_offset_) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControl initial pose validation failed: lateral offset %.3f m", e_d);
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
  }

  // Walk velocity smoother state (0 since robot is stationary)
  double prev_cmd_vx = 0.0;

  // ── Phase 1-3: Trapezoidal profile + PD heading control (omega-only) ──
  amr_motion_control::TrapezoidalProfile profile(
    target_distance, goal->max_linear_speed, goal->acceleration);

  // Reset localization watchdog for this run
  watchdog_->reset();

  bool reached = false;
  int tf_fail_count = 0;
  const int tf_fail_max = 20;  // 1 second at 20Hz -> abort

  // Track last known projection/errors for abort reporting
  double last_projection = 0.0;
  double last_e_d = 0.0;
  double last_e_theta_deg = 0.0;

  while (rclcpp::ok() && !reached) {
    // ── Read position from /robot_pose (map->base_link) ──
    if (lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock())) {
      tf_fail_count = 0;
      watchdog_->updatePose(robot_x, robot_y, robot_yaw);
    } else {
      tf_fail_count++;
      if (tf_fail_count >= tf_fail_max) {
        RCLCPP_ERROR(node_->get_logger(),
          "TF2 lookup failed %d consecutive times at dist=%.3f m",
          tf_fail_count, last_projection);
        finish_abort(-4, last_projection, last_e_d, last_e_theta_deg, start_time);
        return;
      }
      // else: use last known position for this cycle
    }

    // Cancel check
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "YawControl cancelled at dist=%.3f m", last_projection);
      finish_abort(-1, last_projection, last_e_d, last_e_theta_deg, start_time);
      return;
    }

    // Global timeout check
    if ((node_->now() - start_time).seconds() > max_timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControl global timeout (%.1f s), dist=%.3f m", max_timeout_sec_, last_projection);
      finish_abort(-3, last_projection, last_e_d, last_e_theta_deg, start_time);
      return;
    }

    // Localization watchdog
    if (enable_localization_watchdog_ && !watchdog_->checkHealth()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Localization health check failed at dist=%.3f m", last_projection);
      finish_abort(-4, last_projection, last_e_d, last_e_theta_deg, start_time);
      return;
    }

    // Safety checks
    if (safety_sub_->is_dangerous()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Safety DANGEROUS at dist=%.3f m, aborting yaw_control", last_projection);
      finish_abort(amr_safety_core::abort_codes::SAFETY_DANGEROUS,
        last_projection, last_e_d, last_e_theta_deg, start_time);
      return;
    }
    if (!safety_sub_->pose_valid()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Safety pose invalid at dist=%.3f m, aborting yaw_control", last_projection);
      finish_abort(-4, last_projection, last_e_d, last_e_theta_deg, start_time);
      return;
    }

    // Compute projection along path
    double rx = robot_x - goal->start_x;
    double ry = robot_y - goal->start_y;
    double projection = rx * ux + ry * uy;

    // ── Arrival check ──
    double remaining = target_distance - projection;
    double clamped_projection = std::max(0.0, projection);
    auto prof_out = profile.getSpeed(clamped_projection);
    double vx_profile = prof_out.speed;
    watchdog_->setCurrentSpeed(vx_profile);

    // min_vx clamp: prevent deadlock from projection<0
    bool near_goal = (remaining < goal_reach_threshold_);
    if (prof_out.phase != amr_motion_control::ProfilePhase::DONE && !near_goal
        && vx_profile < min_vx_)
    {
      vx_profile = min_vx_;
    }

    // Arrival condition: profile DONE OR projection overshoot
    if (prof_out.phase == amr_motion_control::ProfilePhase::DONE
        || projection >= target_distance)
    {
      reached = true;
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      break;
    }

    // ── Inlined PathController update (Mode A, vy=0) ──
    const double e_d = rx * uy - ry * ux;
    // 고속 IMU yaw 를 map-frame 으로 융합해 heading 오차 계산 (위치/CTE 는 robot_x/robot_y 만 사용)
    const double robot_yaw_ctrl = fuseImuYawToMap(last_yaw_rad_.load(), imu_yaw_offset);
    const double e_theta_raw = normalizeAngle(robot_yaw_ctrl - theta_path);

    // Recursive moving average filter for e_theta
    filter_sum -= filter_buf[filter_idx];
    filter_buf[filter_idx] = e_theta_raw;
    filter_sum += e_theta_raw;
    filter_idx = (filter_idx + 1) % heading_filter_window_;
    if (filter_count < heading_filter_window_) { ++filter_count; }
    const double e_theta = filter_sum / filter_count;

    double de_theta = 0.0;
    if (dt > 0.0) {
      de_theta = (e_theta - prev_e_theta) / dt;
    }

    // PD heading control → omega
    double omega = -Kp_heading_ * e_theta - Kd_heading_ * de_theta;
    // Clamp to max_omega
    if (omega >  max_omega_) { omega =  max_omega_; }
    if (omega < -max_omega_) { omega = -max_omega_; }

    // Min turning radius constraint (omega <= vx / r_min)
    if (vx_profile > 1e-6) {
      double omega_limit = vx_profile / min_turning_radius_;
      if (omega >  omega_limit) { omega =  omega_limit; }
      if (omega < -omega_limit) { omega = -omega_limit; }
    }

    // Angular acceleration limit (smooth omega transitions)
    if (dt > 0.0) {
      double max_omega_change = alpha_max_ * dt;
      double omega_diff = omega - prev_omega_pc;
      if (std::fabs(omega_diff) > max_omega_change) {
        omega = prev_omega_pc + std::copysign(max_omega_change, omega_diff);
      }
    }
    prev_omega_pc = omega;
    prev_e_theta = e_theta;

    // ── Inlined TransientGuard: omega rate limit only ──
    {
      double diff = omega - prev_omega_tg;
      if (std::fabs(diff) > omega_rate_limit_) {
        omega = prev_omega_tg + std::copysign(omega_rate_limit_, diff);
      }
      prev_omega_tg = omega;
    }

    // Walk velocity profile: smooth vx changes to prevent motor jerks
    {
      double acc_step = walk_accel_limit_ * dt;
      double dec_step = walk_decel_limit_ * dt;
      auto velProfile = [](double cur, double tgt, double a_step, double d_step) -> double {
        if (std::fabs(tgt) < 0.01) {
          if (cur > d_step) { return cur - d_step; }
          if (cur < -d_step) { return cur + d_step; }
          return tgt;
        }
        if (tgt > cur) { return std::fmin(tgt, cur + a_step); }
        if (tgt < cur) { return std::fmax(tgt, cur - a_step); }
        return tgt;
      };
      vx_profile = velProfile(prev_cmd_vx, vx_profile, acc_step, dec_step);
      prev_cmd_vx = vx_profile;
    }

    publishCmdVel(cmd_vel_pub_, vx_profile, omega);

    // Update last known state for abort reporting
    last_projection = projection;
    last_e_d = e_d;
    last_e_theta_deg = e_theta * 180.0 / M_PI;

    // Debug log at 10 Hz (every 2 cycles at 20 Hz)
    static int dbg_cnt = 0;
    if (++dbg_cnt % 2 == 0) {
      RCLCPP_INFO(node_->get_logger(),
        "[YawCtrl] vx=%.3f omega=%.4f proj=%.3fm lat=%.3fm hdg=%.1f°",
        vx_profile, omega, projection, e_d, e_theta * 180.0 / M_PI);
    }

    // Feedback
    uint8_t phase_id;
    switch (prof_out.phase) {
      case amr_motion_control::ProfilePhase::ACCEL:  phase_id = 1; break;
      case amr_motion_control::ProfilePhase::CRUISE: phase_id = 2; break;
      case amr_motion_control::ProfilePhase::DECEL:  phase_id = 3; break;
      default:                                       phase_id = 3; break;
    }
    feedback->phase = phase_id;
    feedback->current_distance = projection;
    feedback->current_lateral_error = e_d;
    feedback->current_heading_error = e_theta * 180.0 / M_PI;
    feedback->current_vx = vx_profile;
    feedback->current_vy = 0.0;
    feedback->current_omega = omega;
    feedback->w1_drive_rpm = 0.0;
    feedback->w2_drive_rpm = 0.0;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // Stop driving
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
  watchdog_->setCurrentSpeed(0.0);

  // Read final state from /robot_pose
  lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock());
  {
    double rx = robot_x - goal->start_x;
    double ry = robot_y - goal->start_y;
    last_projection = rx * ux + ry * uy;
    last_e_d = rx * uy - ry * ux;
    last_e_theta_deg = normalizeAngle(robot_yaw - theta_path) * 180.0 / M_PI;
  }

  RCLCPP_INFO(node_->get_logger(),
    "YawControl Phase 1-3 complete: dist=%.3f m, lat_err=%.4f m, head_err=%.2f deg",
    last_projection, last_e_d, last_e_theta_deg);

  // Clear path visualization on completion
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = node_->now();
    path_viz_pub_->publish(empty);
  }

  // ── Success ──
  result->status = 0;
  result->actual_distance = last_projection;
  result->final_lateral_error = last_e_d;
  result->final_heading_error = last_e_theta_deg;
  result->elapsed_time = (node_->now() - start_time).seconds();

  goal_handle->succeed(result);

  RCLCPP_INFO(node_->get_logger(),
    "YawControl complete: dist=%.3f/%.3f m, lat_err=%.4f m, head_err=%.2f deg, time=%.1f s",
    last_projection, target_distance, last_e_d, last_e_theta_deg, result->elapsed_time);
}

// ════════════════════════════════════════════════════════
//  Publish / Callbacks
// ════════════════════════════════════════════════════════

void YawControlActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_ = true;
}

}  // namespace amr_motion_control_2wd
