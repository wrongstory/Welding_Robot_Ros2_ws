#include "amr_motion_control_2wd/turn_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_safety_core/abort_codes.hpp"

#include <chrono>
#include <thread>

namespace amr_motion_control_2wd
{

using namespace std::placeholders;

// ── 생성자 ──
// 원본 대비 제거: DualSteerIK, wheel_state_sub_, motor_status_sub_,
//   w1_x/y, w2_x/y, wheel_radius, gear_walk, steer_tolerance, steer_timeout
// 원본 대비 변경: wheel_motor_cmd → cmd_vel (Twist)
TurnActionServer::TurnActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  control_rate_hz_ = safeParam(node_, "control_rate_hz", 50.0);

  // Turn precision parameters (shared with Spin)
  double deadband_deg = safeParam(node_, "imu_deadband_deg", 0.05);
  imu_deadband_rad_ = deadband_deg * M_PI / 180.0;
  min_speed_dps_ = safeParam(node_, "min_speed_dps", 2.0);
  fine_correction_threshold_deg_ = safeParam(node_, "fine_correction_threshold_deg", 0.3);
  fine_correction_speed_dps_ = safeParam(node_, "fine_correction_speed_dps", 3.0);
  fine_correction_timeout_sec_ = safeParam(node_, "fine_correction_timeout_sec", 3.0);
  settling_delay_ms_ = safeParam(node_, "settling_delay_ms", 200);

  // Publisher: cmd_vel (Twist) — goes through tc_motion_guard safety chain
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10));

  // Subscriber: IMU
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&TurnActionServer::imuCallback, this, _1));

  // Action server
  action_server_ = rclcpp_action::create_server<Turn>(
    node_, "amr_turn_action",
    std::bind(&TurnActionServer::handle_goal, this, _1, _2),
    std::bind(&TurnActionServer::handle_cancel, this, _1),
    std::bind(&TurnActionServer::handle_accepted, this, _1));

  // SafetySubscriber
  {
    amr_safety_core::SafetySubscriberConfig cfg;
    safety_sub_ = std::make_unique<amr_safety_core::SafetySubscriber>(node_.get(), cfg);
  }

  RCLCPP_INFO(node_->get_logger(), "TurnActionServer (2WD) initialized");
}

// ── handle_goal ──
// 원본과 동일 (target_angle=0, turn_radius<=0, max_linear_speed<=0, accel_angle<=0 → reject)
// hold_steer/exit_steer_angle 검증 제거: 2WD에서 무시
rclcpp_action::GoalResponse TurnActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Turn::Goal> goal)
{
  if (std::abs(goal->target_angle) < 1e-6) {
    RCLCPP_WARN(node_->get_logger(), "Turn rejected: target_angle is 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->turn_radius <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Turn rejected: turn_radius <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->max_linear_speed <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Turn rejected: max_linear_speed <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->accel_angle <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Turn rejected: accel_angle <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Mutual exclusion (전 서버 공유)
  auto expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::TURN)) {
    RCLCPP_WARN(node_->get_logger(), "Turn rejected: %s is running", to_string(expected));
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_->get_logger(), "Turn goal accepted: %.1f deg, R=%.2f m",
    goal->target_angle, goal->turn_radius);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ── handle_cancel ──
// 원본과 동일
rclcpp_action::CancelResponse TurnActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleTurn> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "Turn cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ── handle_accepted ──
// 원본과 동일
void TurnActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTurn> goal_handle)
{
  try {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  } catch (const std::system_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn execute thread: %s", e.what());
    g_active_action.store(ActiveAction::NONE);
    auto result = std::make_shared<Turn::Result>();
    result->status = -5;
    goal_handle->abort(result);
  }
}

// ── execute ──
// 원본 대비 제거: Phase 0 (steer align), Phase 4 (steer return), DualSteerIK
// 원본 대비 변경: IK::compute({v, 0, omega}) → publishCmdVel(v, omega)
//                IK::computeSpin(omega) → publishCmdVel(0, omega)
void TurnActionServer::execute(const std::shared_ptr<GoalHandleTurn> goal_handle)
{
  ActionGuard guard;  // 소멸자가 자동으로 g_active_action = NONE
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Turn::Feedback>();
  auto result = std::make_shared<Turn::Result>();

  rclcpp::Rate rate(control_rate_hz_);

  // Direction: + CCW, - CW
  const double sign = (goal->target_angle >= 0.0) ? 1.0 : -1.0;
  const double target_abs = std::abs(goal->target_angle);       // deg
  const double turn_radius = static_cast<double>(goal->turn_radius);
  const double max_v = static_cast<double>(goal->max_linear_speed);
  const double accel_angle = static_cast<double>(goal->accel_angle);

  // Convert linear speed → angular speed (spec §5.1)
  const double max_omega_rad = max_v / turn_radius;              // rad/s
  const double max_omega_deg = max_omega_rad * 180.0 / M_PI;     // deg/s

  // accel_angle(deg) → acceleration(deg/s²) conversion (spec §5.1)
  // v² = 2*a*d → a = v²/(2*d)
  const double accel_dps2 = (max_omega_deg * max_omega_deg) / (2.0 * accel_angle);

  auto start_time = node_->now();
  double accumulated_angle = 0.0;  // deg

  // Phase 0 (steer align) — 2WD에서 제거: 조향 모터 없음, 즉시 구동

  // ── IMU 수신 확인 ──
  if (!imu_received_) {
    RCLCPP_ERROR(node_->get_logger(), "IMU data not received, aborting turn");
    publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    result->status = -3;
    result->actual_angle = 0.0;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->abort(result);
    return;
  }

  // ── Phase 1-3: Trapezoidal profile (accel/cruise/decel) ──
  amr_motion_control::TrapezoidalProfile profile(target_abs, max_omega_deg, accel_dps2);

  // IMU-based angle tracking: record previous yaw
  double prev_yaw = last_yaw_rad_.load();

  while (rclcpp::ok() && !profile.isComplete(accumulated_angle)) {
    if (goal_handle->is_canceling()) {
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = -1;
      result->actual_angle = sign * accumulated_angle;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->canceled(result);
      return;
    }

    // Safety abort checks
    if (safety_sub_->is_dangerous()) {
      RCLCPP_ERROR(node_->get_logger(), "Turn aborted: DANGEROUS safety state");
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = amr_safety_core::abort_codes::SAFETY_DANGEROUS;
      result->actual_angle = sign * accumulated_angle;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->abort(result);
      return;
    }
    if (!safety_sub_->pose_valid()) {
      RCLCPP_ERROR(node_->get_logger(), "Turn aborted: pose invalid");
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = -4;
      result->actual_angle = sign * accumulated_angle;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->abort(result);
      return;
    }

    // Profile → angular speed
    auto prof_out = profile.getSpeed(accumulated_angle);
    double omega_dps = prof_out.speed;  // deg/s (always >= 0)
    // Min speed clamp: ensure motor responds during decel phase
    if (prof_out.phase != amr_motion_control::ProfilePhase::DONE && omega_dps < min_speed_dps_) {
      omega_dps = min_speed_dps_;
    }
    double omega_rad = omega_dps * M_PI / 180.0;  // rad/s

    // Restore linear speed: v = ω * R
    double v = omega_rad * turn_radius;

    // 2WD: R-turn uses publishCmdVel(v, sign*omega)
    publishCmdVel(cmd_vel_pub_, v, sign * omega_rad);

    // IMU-based angle tracking with deadband filter
    double delta_deg = readImuDelta(last_yaw_rad_.load(), prev_yaw, imu_deadband_rad_);
    accumulated_angle += std::abs(delta_deg);  // Pattern B

    // Feedback
    uint8_t phase_id;
    switch (prof_out.phase) {
      case amr_motion_control::ProfilePhase::ACCEL:  phase_id = 1; break;
      case amr_motion_control::ProfilePhase::CRUISE: phase_id = 2; break;
      case amr_motion_control::ProfilePhase::DECEL:  phase_id = 3; break;
      default:                                       phase_id = 3; break;
    }
    feedback->phase = phase_id;
    feedback->current_angle = sign * accumulated_angle;
    feedback->current_linear_speed = v;
    feedback->current_angular_speed = sign * omega_dps;
    feedback->remaining_angle = target_abs - accumulated_angle;
    feedback->w1_drive_rpm = 0.0;
    feedback->w2_drive_rpm = 0.0;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // ── Settling Delay ──
  // Phase 1-3 ends at min_speed_dps (not zero). Wait for physical stop before fine correction.
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
  rclcpp::sleep_for(std::chrono::milliseconds(settling_delay_ms_));

  // IMU update: reflect any inertial rotation during settling
  {
    double delta_deg = readImuDelta(last_yaw_rad_.load(), prev_yaw, imu_deadband_rad_);
    if (delta_deg != 0.0) {  // Pattern C
      if (sign * delta_deg > 0.0) {
        accumulated_angle += std::abs(delta_deg);
      } else {
        accumulated_angle -= std::abs(delta_deg);
        if (accumulated_angle < 0.0) accumulated_angle = 0.0;
      }
    }
  }

  // ── Phase 3.5: Fine Correction ──
  // 원본 대비 변경: IK::computeSpin(omega) → publishCmdVel(0, omega)
  // steer re-align 루프 제거: 2WD에서 조향 모터 없음
  double angle_error = target_abs - accumulated_angle;

  if (std::abs(angle_error) > fine_correction_threshold_deg_) {
    auto fine_start = node_->now();
    double fine_omega_dps = fine_correction_speed_dps_;

    while (rclcpp::ok() && std::abs(angle_error) > fine_correction_threshold_deg_) {
      if (goal_handle->is_canceling()) {
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        result->status = -1;
        result->actual_angle = sign * accumulated_angle;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->canceled(result);
        return;
      }

      // Fine correction timeout
      if ((node_->now() - fine_start).seconds() > fine_correction_timeout_sec_) {
        RCLCPP_WARN(node_->get_logger(), "Turn fine correction timeout, error=%.2f deg", angle_error);
        break;
      }

      // Safety abort checks
      if (safety_sub_->is_dangerous()) {
        RCLCPP_ERROR(node_->get_logger(), "Turn aborted: DANGEROUS safety state");
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        result->status = amr_safety_core::abort_codes::SAFETY_DANGEROUS;
        result->actual_angle = sign * accumulated_angle;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->abort(result);
        return;
      }
      if (!safety_sub_->pose_valid()) {
        RCLCPP_ERROR(node_->get_logger(), "Turn aborted: pose invalid");
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        result->status = -4;
        result->actual_angle = sign * accumulated_angle;
        result->elapsed_time = (node_->now() - start_time).seconds();
        goal_handle->abort(result);
        return;
      }

      // Determine correction direction
      double correction_sign = (angle_error > 0.0) ? sign : -sign;
      double omega_rad = correction_sign * fine_omega_dps * M_PI / 180.0;
      publishCmdVel(cmd_vel_pub_, 0.0, omega_rad);

      // IMU tracking (direction-aware accumulation)
      double delta_deg = readImuDelta(last_yaw_rad_.load(), prev_yaw, imu_deadband_rad_);
      if (delta_deg != 0.0) {  // Pattern C
        if (sign * delta_deg > 0.0) {
          accumulated_angle += std::abs(delta_deg);
        } else {
          accumulated_angle -= std::abs(delta_deg);
          if (accumulated_angle < 0.0) accumulated_angle = 0.0;
        }
      }

      angle_error = target_abs - accumulated_angle;

      // Feedback (phase 3 maintained - fine correction is sub-phase of decel)
      feedback->phase = 3;
      feedback->current_angle = sign * accumulated_angle;
      feedback->current_linear_speed = 0.0;
      feedback->current_angular_speed = sign * fine_omega_dps * (angle_error > 0 ? 1.0 : -1.0);
      feedback->remaining_angle = angle_error;
      feedback->w1_drive_rpm = 0.0;
      feedback->w2_drive_rpm = 0.0;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }
  }

  // Final stop
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);

  // Phase 4 (steer return) — 2WD에서 제거: 조향 모터 없음, hold_steer/exit_steer_angle 무시

  // Success
  result->status = 0;
  result->actual_angle = sign * accumulated_angle;
  result->elapsed_time = (node_->now() - start_time).seconds();
  goal_handle->succeed(result);

  double final_error = target_abs - accumulated_angle;
  RCLCPP_INFO(node_->get_logger(),
    "Turn complete: target=%.1f, actual=%.1f, error=%.2f deg, R=%.2f m, time=%.1f s",
    goal->target_angle, sign * accumulated_angle, final_error,
    goal->turn_radius, result->elapsed_time);
  if (std::abs(final_error) > 2.0) {
    RCLCPP_WARN(node_->get_logger(),
      "Turn precision warning: final error %.2f deg exceeds 2.0 deg threshold", final_error);
  }
}

// ── imuCallback ──
// 원본 cpp와 동일
void TurnActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
