#include "amr_motion_control_2wd/spin_action_server.hpp"
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
SpinActionServer::SpinActionServer(rclcpp::Node::SharedPtr node)
: node_(node),
  pose_cache_(node.get())
{
  control_rate_hz_ = safeParam(node_, "control_rate_hz", 50.0);

  // Spin precision parameters
  double deadband_deg = safeParam(node_, "imu_deadband_deg", 0.05);
  imu_deadband_rad_ = deadband_deg * M_PI / 180.0;
  min_speed_dps_ = safeParam(node_, "min_speed_dps", 2.0);
  fine_correction_threshold_deg_ = safeParam(node_, "fine_correction_threshold_deg", 0.3);
  fine_correction_speed_dps_ = safeParam(node_, "fine_correction_speed_dps", 3.0);
  fine_correction_timeout_sec_ = safeParam(node_, "fine_correction_timeout_sec", 3.0);
  settling_delay_ms_ = safeParam(node_, "settling_delay_ms", 200);

  // Fine-correction PID gains (Phase 3.5). Default kp=1.5/ki=0/kd=0.5 (TR_Nav-tuned).
  kp_spin_            = safeParam(node_, "kp_spin", 1.5);
  ki_spin_            = safeParam(node_, "ki_spin", 0.0);
  kd_spin_            = safeParam(node_, "kd_spin", 0.5);
  integral_limit_deg_ = safeParam(node_, "integral_limit_deg", 30.0);

  // Coarse 조기종료 band [deg] (overshoot 대책). coarse(사다리꼴)를 (target - band)에서 종료시켜,
  // 정지 명령 후 관성 coast(~band)가 목표 "앞"에서 일어나게 함 → 정점이 목표 근처 → overshoot 상쇄.
  // 0 = 기존(목표까지 coarse 구동) 동작. 남은 잔류는 Phase 3.5 fine PID 가 마무리.
  coarse_exit_band_deg_ = safeParam(node_, "coarse_exit_band_deg", 0.0);

  // Publisher: cmd_vel (Twist) — goes through tc_motion_guard safety chain
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10));

  // Subscriber: IMU
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&SpinActionServer::imuCallback, this, _1));

  // Action server
  action_server_ = rclcpp_action::create_server<Spin>(
    node_, "amr_spin_action",
    std::bind(&SpinActionServer::handle_goal, this, _1, _2),
    std::bind(&SpinActionServer::handle_cancel, this, _1),
    std::bind(&SpinActionServer::handle_accepted, this, _1));

  // SafetySubscriber
  {
    amr_safety_core::SafetySubscriberConfig cfg;
    safety_sub_ = std::make_unique<amr_safety_core::SafetySubscriber>(node_.get(), cfg);
  }

  // Idle keep-alive: goal 없을 때(g_active_action==NONE) cmd_vel=0 을 20Hz 로 발행 →
  // ESP32 가 cmd_vel stale 로 HARD_STOP 되지 않고 st:5(RUNNING) 유지 → 매 테스트 재투입/재enable 불필요.
  idle_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      if (g_active_action.load() == ActiveAction::NONE) {
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      }
    });

  RCLCPP_INFO(node_->get_logger(), "SpinActionServer (2WD) initialized");
}

// ── handle_goal ──
// target_angle: 절대 각도 (map frame). 0도 포함 모든 값 허용.
// speed/accel만 검증.
rclcpp_action::GoalResponse SpinActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Spin::Goal> goal)
{
  if (goal->max_angular_speed <= 0.0 || goal->angular_acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Rejected: invalid speed/acceleration");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Mutual exclusion (전 서버 공유)
  auto expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::SPIN)) {
    RCLCPP_WARN(node_->get_logger(), "Spin rejected: %s is running", to_string(expected));
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(node_->get_logger(), "Spin goal accepted: target=%.1f deg (absolute)", goal->target_angle);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ── handle_cancel ──
// 원본과 동일
rclcpp_action::CancelResponse SpinActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleSpin> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "Spin cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ── handle_accepted ──
// 원본과 동일
void SpinActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleSpin> goal_handle)
{
  try {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  } catch (const std::system_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn execute thread: %s", e.what());
    g_active_action.store(ActiveAction::NONE);
    auto result = std::make_shared<Spin::Result>();
    result->status = -5;
    goal_handle->abort(result);
  }
}

// ── execute ──
// 절대 각도 모드: tf(map→base_link) yaw로 목표 회전량 계산, IMU 상대 delta로 정밀 제어
void SpinActionServer::execute(const std::shared_ptr<GoalHandleSpin> goal_handle)
{
  ActionGuard guard;  // 소멸자가 자동으로 g_active_action = NONE
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Spin::Feedback>();
  auto result = std::make_shared<Spin::Result>();

  rclcpp::Rate rate(control_rate_hz_);
  auto start_time = node_->now();

  // ── 런타임 게인 재read (ros2 param set 으로 노드 재시작 없이 변경 — 재투입 없이 튜닝) ──
  kp_spin_            = safeParam(node_, "kp_spin", kp_spin_);
  ki_spin_            = safeParam(node_, "ki_spin", ki_spin_);
  kd_spin_            = safeParam(node_, "kd_spin", kd_spin_);
  integral_limit_deg_ = safeParam(node_, "integral_limit_deg", integral_limit_deg_);
  fine_correction_threshold_deg_ = safeParam(node_, "fine_correction_threshold_deg", fine_correction_threshold_deg_);

  // ── 1. tf에서 현재 절대 yaw 읽기 ──
  double start_tf_yaw_deg = 0.0;
  if (!lookupTfYaw(pose_cache_, start_tf_yaw_deg, node_->get_logger(), node_->get_clock())) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read robot pose (/robot_pose), aborting spin");
    publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    result->status = -4;  // tf_fail
    result->actual_angle = start_tf_yaw_deg;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->abort(result);
    return;
  }

  // ── 2. IMU 수신 확인 + 시작 IMU yaw 저장 ──
  if (!imu_received_) {
    RCLCPP_ERROR(node_->get_logger(), "IMU data not received, aborting spin");
    publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    result->status = -3;
    result->actual_angle = start_tf_yaw_deg;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->abort(result);
    return;
  }
  double start_imu_yaw_rad = last_yaw_rad_.load();

  // ── 3. 회전량 계산 (shortest path, [-180, 180]) ──
  double rotation_needed = goal->target_angle - start_tf_yaw_deg;
  // Normalize to [-180, 180]
  while (rotation_needed > 180.0) rotation_needed -= 360.0;
  while (rotation_needed < -180.0) rotation_needed += 360.0;

  RCLCPP_INFO(node_->get_logger(),
    "Spin: tf_start=%.1f, imu_start=%.1f deg, target=%.1f, rotation=%.1f deg",
    start_tf_yaw_deg, start_imu_yaw_rad * 180.0 / M_PI,
    goal->target_angle, rotation_needed);

  // 회전량이 거의 0이면 즉시 성공
  if (std::abs(rotation_needed) < fine_correction_threshold_deg_) {
    result->status = 0;
    result->actual_angle = start_tf_yaw_deg;
    result->elapsed_time = (node_->now() - start_time).seconds();
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Spin: already at target (error=%.2f deg)", rotation_needed);
    return;
  }

  const double sign = (rotation_needed >= 0.0) ? 1.0 : -1.0;
  const double target_abs = std::abs(rotation_needed);  // deg
  const double max_omega_dps = goal->max_angular_speed;

  // 절대 목표 IMU yaw (경계-강건 PID 정밀제어용). PID fine 의 오차는 progress(lift)가 아니라
  // normalizeAngle(target_imu_yaw - cur) 로 계산 → 항상 [-180,180] 연속값이라 ±180 경계
  // (target 180.1 / -180.1, IMU wrap)에서도 부호반전·점프 없이 목표서 0 으로 정밀 정착.
  const double target_imu_yaw_rad = start_imu_yaw_rad + sign * target_abs * M_PI / 180.0;
  auto remaining_signed_deg = [target_imu_yaw_rad](double cur_yaw_rad) -> double {
    return normalizeAngle(target_imu_yaw_rad - cur_yaw_rad) * 180.0 / M_PI;  // [-180,180], 목표서 0
  };

  // 진행량(deg): 등록한 start IMU yaw 대비 변화량을 "명령 회전 방향(sign)"으로 [0,360)에 투영.
  // 적분(델타 누적) 없이 매 주기 단일 샘플로 직접 측정.
  // ※ 단순 음수클램프((d<0)?0:d)는 |회전|=180°에서 normalizeAngle 부호반전 → progress가 0으로
  //   리셋되어 무한회전을 유발했음. 방향을 알고 있으므로 음수일 때 +360으로 lift하면 180을
  //   매끄럽게 넘긴다. 시작 부근 후진/노이즈는 반대편 호로 가서 (target_abs+360)/2 초과 → 0 처리.
  auto measureProgress = [this, start_imu_yaw_rad, sign, target_abs]() -> double {
    double delta = normalizeAngle(last_yaw_rad_.load() - start_imu_yaw_rad) * 180.0 / M_PI;  // [-180,180]
    double prog = sign * delta;                          // 명령 방향 진행량
    if (prog < 0.0) { prog += 360.0; }                   // 방향 인식 lift → [0,360)
    if (prog > (target_abs + 360.0) * 0.5) { prog = 0.0; }  // 반대편 호 = 시작부근 후진/노이즈 → 0
    return prog;
  };
  double progress_deg = measureProgress();

  // ── 순수 PID 제어 (사다리꼴 프로파일 완전 제거, 2026-06-08 사용자 요청) ──
  // 회전 전체를 단일 PID 폐루프로: omega_dps = Kp·e + Ki·∫e + Kd·ė.
  //   e = remaining_signed_deg = normalizeAngle(target_imu_yaw - cur) ∈[-180,180] (목표서 0, ±180 경계 강건).
  // 시작 시 |e| 크므로 omega 가 max_omega_dps 로 포화(→정지마찰 돌파 출발), e 줄며 비례 감속, e→0 정착.
  //   → 사다리꼴/coarse/min_speed floor/coarse_exit_band/settling delay 없음. PID 가 가감속을 전부 수행.
  // 부호는 e 가 직접 인코딩(e>0 → CCW) → omega_rad = omega_dps (sign 곱 불필요).
  const double dt = 1.0 / control_rate_hz_;
  double integral = 0.0;
  double angle_error = remaining_signed_deg(last_yaw_rad_.load());  // 목표까지 부호오차[deg]
  double prev_error = angle_error;
  // 전체 타임아웃: 진행시간(target/max_ω)×4, 최소 5s (정지마찰 정착 여유).
  const double pid_timeout_sec = std::max(5.0, 4.0 * target_abs / std::max(1.0, max_omega_dps));
  // |e|≤threshold 가 settling_delay 만큼 지속되면 정착 완료로 종료.
  const int settled_need = std::max(1, static_cast<int>(0.001 * settling_delay_ms_ * control_rate_hz_));
  int settled_cycles = 0;
  auto pid_start = node_->now();

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = -1;
      result->actual_angle = start_tf_yaw_deg + sign * progress_deg;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->canceled(result);
      return;
    }

    if ((node_->now() - pid_start).seconds() > pid_timeout_sec) {
      RCLCPP_WARN(node_->get_logger(), "Spin PID timeout (%.1fs), error=%.2f deg",
                  pid_timeout_sec, angle_error);
      break;
    }

    // Safety abort checks
    if (safety_sub_->is_dangerous()) {
      RCLCPP_ERROR(node_->get_logger(), "Spin aborted: DANGEROUS safety state");
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = amr_safety_core::abort_codes::SAFETY_DANGEROUS;
      result->actual_angle = start_tf_yaw_deg + sign * progress_deg;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->abort(result);
      return;
    }
    if (!safety_sub_->pose_valid()) {
      RCLCPP_ERROR(node_->get_logger(), "Spin aborted: pose invalid");
      publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
      result->status = -4;
      result->actual_angle = start_tf_yaw_deg + sign * progress_deg;
      result->elapsed_time = (node_->now() - start_time).seconds();
      goal_handle->abort(result);
      return;
    }

    // 목표 yaw 직접 부호오차 (경계-강건). progress 는 feedback/result 표시용만.
    angle_error = remaining_signed_deg(last_yaw_rad_.load());
    progress_deg = measureProgress();

    // ── PID (conditional integration anti-windup) ──
    const double derivative = (angle_error - prev_error) / dt;
    const double omega_pd = kp_spin_ * angle_error + kd_spin_ * derivative;
    // 적분 관리:
    //  ① deadband 진입(|e|≤threshold) 또는 부호반전(zero-crossing) → reset (windup carryover/kick 방지).
    //  ② PD 출력 포화(|omega_pd|≥max_w, 즉 cruise 구간) 시 적분 정지 — 비포화 구간(목표 접근)서만 누적.
    //     → 긴 cruise 동안 적분 windup 으로 감속 시 overshoot 하는 것을 막음 (순수 PID 의 핵심 anti-windup).
    if (std::abs(angle_error) <= fine_correction_threshold_deg_ ||
        angle_error * prev_error < 0.0) {
      integral = 0.0;
    } else if (std::fabs(omega_pd) < max_omega_dps) {
      integral += angle_error * dt;
      integral = std::max(-integral_limit_deg_, std::min(integral_limit_deg_, integral));
    }
    prev_error = angle_error;
    double omega_dps = omega_pd + ki_spin_ * integral;

    // |omega| 상한만 clamp (하한 floor 없음 — 순수 PID). e 가 부호 인코딩(e>0 → CCW).
    const double dir = (omega_dps >= 0.0) ? 1.0 : -1.0;
    omega_dps = dir * std::min(max_omega_dps, std::fabs(omega_dps));
    publishCmdVel(cmd_vel_pub_, 0.0, omega_dps * M_PI / 180.0);

    feedback->phase = 1;  // 순수 PID 단일 단계
    feedback->current_angle = start_tf_yaw_deg + sign * progress_deg;
    feedback->current_speed = omega_dps;
    goal_handle->publish_feedback(feedback);

    // 정착 판정 (조기 종료)
    if (std::abs(angle_error) <= fine_correction_threshold_deg_) {
      if (++settled_cycles >= settled_need) { break; }
    } else {
      settled_cycles = 0;
    }

    rate.sleep();
  }

  // Final stop
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);

  // Success
  double final_absolute = start_tf_yaw_deg + sign * progress_deg;
  result->status = 0;
  result->actual_angle = final_absolute;
  result->elapsed_time = (node_->now() - start_time).seconds();
  goal_handle->succeed(result);

  double final_error = remaining_signed_deg(last_yaw_rad_.load());  // 목표 yaw 직접 부호오차 (경계-강건)
  RCLCPP_INFO(node_->get_logger(),
    "Spin complete: target=%.1f, actual=%.1f (abs), rotation=%.2f/%.2f deg, error=%.2f deg, time=%.1f s",
    goal->target_angle, final_absolute, sign * progress_deg, rotation_needed,
    final_error, result->elapsed_time);
  if (std::abs(final_error) > 2.0) {
    RCLCPP_WARN(node_->get_logger(),
      "Spin precision warning: final error %.2f deg exceeds 2.0 deg threshold", final_error);
  }
}

// ── imuCallback ──
void SpinActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
