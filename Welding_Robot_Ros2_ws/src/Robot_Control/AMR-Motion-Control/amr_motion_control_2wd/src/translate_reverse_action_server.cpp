#include "amr_motion_control_2wd/translate_reverse_action_server.hpp"
#include "amr_motion_control_2wd/motion_common.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>

namespace amr_motion_control_2wd
{

using namespace std::placeholders;

// ════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════

TranslateReverseActionServer::TranslateReverseActionServer(rclcpp::Node::SharedPtr node)
: node_(node), pose_cache_(node_.get()), e_theta_filter_(5)
{
  // ── Common control parameters ──
  control_rate_hz_ = safeParam(node_, "control_rate_hz", 50.0);

  // ── TranslateReverse-specific parameters ──
  Kp_heading_            = safeParam(node_,"translate_reverse_Kp_heading", 1.0);
  Kd_heading_            = safeParam(node_,"translate_reverse_Kd_heading", 0.3);
  K_stanley_             = safeParam(node_,"translate_reverse_K_stanley", 2.0);
  K_soft_                = safeParam(node_,"translate_reverse_K_soft", 1.0);
  max_omega_             = safeParam(node_,"translate_reverse_max_omega", 1.0);
  double alpha_max       = safeParam(node_,"translate_reverse_alpha_max", 0.5);
  alpha_max_             = alpha_max;
  double heading_threshold_deg = safeParam(node_,"translate_reverse_heading_threshold_deg", 45.0);
  heading_threshold_     = heading_threshold_deg * M_PI / 180.0;
  max_lateral_offset_    = safeParam(node_,"translate_reverse_max_lateral_offset", 1.0);
  int heading_filter_window = safeParam(node_,"translate_reverse_heading_filter_window", 5);
  heading_filter_window_ = heading_filter_window;
  // Re-initialize filter with configured window size
  e_theta_filter_ = amr_motion_control::RecursiveMovingAverage(heading_filter_window_);

  goal_reach_threshold_      = safeParam(node_,"translate_reverse_goal_reach_threshold", 0.05);
  arrival_tolerance_         = safeParam(node_,"translate_reverse_arrival_tolerance", 0.03);
  min_vx_                    = safeParam(node_,"translate_reverse_min_vx", 0.02);
  behind_start_speed_        = safeParam(node_,"translate_reverse_behind_start_speed", 0.2);
  // behind-start 정책: false=시작점까지 behind_start_speed_ 고정 접근(기존), true=시작점 뒤부터 가속
  behind_start_accelerate_   = safeParam(node_,"translate_reverse_behind_start_accelerate", false);
  behind_start_kick_         = safeParam(node_,"translate_reverse_behind_start_kick", 0.1);
  max_timeout_sec_           = safeParam(node_,"translate_reverse_max_timeout_sec", 60.0);
  localization_timeout_sec_  = safeParam(node_,"translate_reverse_localization_timeout_sec", 2.0);
  position_jump_threshold_   = safeParam(node_,"translate_reverse_position_jump_threshold", 0.3);
  enable_localization_watchdog_ = safeParam(node_,"translate_reverse_enable_localization_watchdog", true);
  walk_accel_limit_          = safeParam(node_,"translate_reverse_walk_accel_limit", 0.5);
  walk_decel_limit_          = safeParam(node_,"translate_reverse_walk_decel_limit", 1.0);

  // 도착 2단 감속 캡 + 부드러운 정지 (T-AMR rosbag 측정 기반)
  approach_gain_             = safeParam(node_,"translate_reverse_approach_gain", 1.5);
  stop_speed_                = safeParam(node_,"translate_reverse_stop_speed", 0.2);
  stop_decel_                = safeParam(node_,"translate_reverse_stop_decel", 0.1);
  jerk_max_                  = safeParam(node_,"translate_reverse_jerk_max", 1.0);
  a_decel_max_               = safeParam(node_,"translate_reverse_a_decel_max", 0.4);
  should_stop_speed_         = safeParam(node_,"translate_reverse_should_stop_speed", 0.1);

  // ── Localization watchdog ──
  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec = localization_timeout_sec_;
  wd_cfg.fixed_jump_threshold = position_jump_threshold_;
  wd_cfg.velocity_margin = 1.3;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  // ── Publisher / Subscribers ──
  // 위치(projection)·CTE 는 PoseCache(/robot_pose, map-frame), heading 은 imu/data 고속 yaw
  //   (시작 시 1회 IMU↔map 오프셋 융합 — fuseImuYawToMap). watchdog 갱신은 execute 루프의
  //   lookupRobotPose 단일 경로로 통일하여 별도 pose_sub_(이중 /robot_pose 구독) 제거.
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10));
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "translate_reverse_path", rclcpp::QoS(10).transient_local());
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&TranslateReverseActionServer::imuCallback, this, _1));

  // ── Action server ──
  action_server_ = rclcpp_action::create_server<Translate>(
    node_, "amr_translate_reverse_action",
    std::bind(&TranslateReverseActionServer::handle_goal, this, _1, _2),
    std::bind(&TranslateReverseActionServer::handle_cancel, this, _1),
    std::bind(&TranslateReverseActionServer::handle_accepted, this, _1));

  // ── Endpoint update service (velocity continuity) ──
  update_endpoint_srv_ = node_->create_service<amr_interfaces::srv::UpdateTranslateEndpoint>(
    "update_translate_reverse_endpoint",
    std::bind(&TranslateReverseActionServer::onUpdateEndpoint, this,
              std::placeholders::_1, std::placeholders::_2));

  // ── Safety subscriber (speed limit + status) ──
  // translate / spin / yaw_control 과 동일한 SafetySubscriber 추상 사용
  //   (이전의 raw /safety/speed_limit·safety_status 구독 + 매직넘버 status==2 를 대체).
  {
    amr_safety_core::SafetySubscriberConfig safety_cfg;
    safety_sub_ = std::make_unique<amr_safety_core::SafetySubscriber>(node_.get(), safety_cfg);
  }

  RCLCPP_INFO(node_->get_logger(),
    "TranslateReverseActionServer (2WD) initialized (pos=/robot_pose, heading=imu/data fused, loc_watchdog=%s, safety=ON)",
    enable_localization_watchdog_ ? "ON" : "OFF");
}

// ════════════════════════════════════════════════════════
//  Action callbacks
// ════════════════════════════════════════════════════════

rclcpp_action::GoalResponse TranslateReverseActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Translate::Goal> goal)
{
  double dx = goal->end_x - goal->start_x;
  double dy = goal->end_y - goal->start_y;
  double dist = std::hypot(dx, dy);
  if (dist < 1e-6) {
    RCLCPP_WARN(node_->get_logger(), "TranslateReverse rejected: start == end");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->max_linear_speed == 0.0) {
    RCLCPP_WARN(node_->get_logger(), "TranslateReverse rejected: max_linear_speed == 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "TranslateReverse rejected: acceleration <= 0");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Mutual exclusion (전 서버 공유)
  auto expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::TRANSLATE_REVERSE)) {
    RCLCPP_WARN(node_->get_logger(), "TranslateReverse rejected: %s is running", to_string(expected));
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node_->get_logger(),
    "TranslateReverse goal accepted: (%.2f,%.2f)->(%.2f,%.2f), dist=%.3f m, v_max=%.2f m/s",
    goal->start_x, goal->start_y, goal->end_x, goal->end_y,
    dist, goal->max_linear_speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateReverseActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleTranslate> /*goal_handle*/)
{
  RCLCPP_INFO(node_->get_logger(), "TranslateReverse cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateReverseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleTranslate> goal_handle)
{
  try {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  } catch (const std::system_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to spawn execute thread: %s", e.what());
    g_active_action.store(ActiveAction::NONE);
    auto result = std::make_shared<Translate::Result>();
    result->status = -5;
    goal_handle->abort(result);
  }
}

// ════════════════════════════════════════════════════════
//  Main execute loop
// ════════════════════════════════════════════════════════

void TranslateReverseActionServer::execute(
  const std::shared_ptr<GoalHandleTranslate> goal_handle)
{
  ActionGuard guard;  // 소멸자가 자동으로 g_active_action = NONE
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Translate::Feedback>();
  auto result   = std::make_shared<Translate::Result>();

  rclcpp::Rate rate(control_rate_hz_);
  const double dt = 1.0 / control_rate_hz_;

  // ── 런타임 게인 재read (ros2 param set 으로 노드 재시작 없이 튜닝 — spin 과 동일) ──
  Kp_heading_ = safeParam(node_, "translate_reverse_Kp_heading", Kp_heading_);
  Kd_heading_ = safeParam(node_, "translate_reverse_Kd_heading", Kd_heading_);
  K_stanley_  = safeParam(node_, "translate_reverse_K_stanley",  K_stanley_);
  K_soft_     = safeParam(node_, "translate_reverse_K_soft",     K_soft_);
  max_omega_  = safeParam(node_, "translate_reverse_max_omega",  max_omega_);
  alpha_max_  = safeParam(node_, "translate_reverse_alpha_max",  alpha_max_);

  // Helper: abort/cancel with cleanup
  auto finish_abort = [&](int8_t status, double actual_dist, double lat_err,
                          double head_err, const rclcpp::Time & start_time) {
    publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
    watchdog_->setCurrentSpeed(0.0);
    {
      nav_msgs::msg::Path empty;
      empty.header.frame_id = "map";
      empty.header.stamp = node_->now();
      path_viz_pub_->publish(empty);
    }
    result->status             = status;
    result->actual_distance    = actual_dist;
    result->final_lateral_error = lat_err;
    result->final_heading_error = head_err;
    result->elapsed_time       = (node_->now() - start_time).seconds();
    if (status == -1) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  };

  auto start_time = node_->now();

  // ── Reverse mode detection ──
  is_reverse_ = (goal->max_linear_speed < 0);
  RCLCPP_INFO(node_->get_logger(),
    "TranslateReverse execute: is_reverse=%s, abs_speed=%.2f m/s",
    is_reverse_ ? "true" : "false", std::fabs(goal->max_linear_speed));

  // ── Setup path (inline PathController) ──
  path_start_x_ = goal->start_x;
  path_start_y_ = goal->start_y;
  path_end_x_   = goal->end_x;
  path_end_y_   = goal->end_y;
  theta_path_      = std::atan2(goal->end_y - goal->start_y, goal->end_x - goal->start_x);
  target_distance_ = std::hypot(goal->end_x - goal->start_x, goal->end_y - goal->start_y);
  path_ux_ = std::cos(theta_path_);
  path_uy_ = std::sin(theta_path_);
  prev_e_theta_ = 0.0;
  prev_omega_   = 0.0;
  e_theta_filter_.reset();

  RCLCPP_INFO(node_->get_logger(),
    "TranslateReverse execute: path_angle=%.1f deg, target_dist=%.3f m",
    theta_path_ * 180.0 / M_PI, target_distance_);

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
    RCLCPP_ERROR(node_->get_logger(), "IMU data not received, aborting TranslateReverse");
    finish_abort(-3, 0.0, 0.0, 0.0, start_time);
    return;
  }

  // ── Initial pose from /robot_pose (map-frame, PoseCache) ──
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  if (!lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock())) {
    RCLCPP_ERROR(node_->get_logger(),
      "/robot_pose not available (stale/unreceived), aborting TranslateReverse");
    finish_abort(-3, 0.0, 0.0, 0.0, start_time);
    return;
  }

  // ── IMU↔map yaw 오프셋 1회 등록 (heading 고속 피드백용) ──
  // 이후 heading 오차는 fuseImuYawToMap(imu_yaw, imu_yaw_offset)(+후진 시 π) - theta_path_ 로 계산.
  // 위치(projection)·cross-track error 는 /robot_pose 를 그대로 사용.
  const double imu_yaw_offset = imuYawOffset(robot_yaw, last_yaw_rad_.load());

  // ── Initial pose validation (inline validateInitialPose) ──
  {
    const double rx       = robot_x - path_start_x_;
    const double ry       = robot_y - path_start_y_;
    const double proj_ini = rx * path_ux_ + ry * path_uy_;
    const double e_d_ini  = rx * path_uy_ - ry * path_ux_;
    const double yaw_ini  = is_reverse_ ? (robot_yaw + M_PI) : robot_yaw;
    const double e_th_ini = normalizeAngle(yaw_ini - theta_path_);

    if (proj_ini >= target_distance_) {
      RCLCPP_WARN(node_->get_logger(),
        "TranslateReverse initial pose validation failed: robot already past goal");
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
    if (std::fabs(e_th_ini) > heading_threshold_) {
      RCLCPP_WARN(node_->get_logger(),
        "TranslateReverse initial pose validation failed: heading error=%.1f deg > %.1f deg",
        e_th_ini * 180.0 / M_PI, heading_threshold_ * 180.0 / M_PI);
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
    if (std::fabs(e_d_ini) > max_lateral_offset_) {
      RCLCPP_WARN(node_->get_logger(),
        "TranslateReverse initial pose validation failed: lateral offset=%.3f m > %.3f m",
        e_d_ini, max_lateral_offset_);
      finish_abort(-2, 0.0, 0.0, 0.0, start_time);
      return;
    }
  }

  // ── Walk velocity smoother state ──
  double prev_cmd_vx = 0.0;
  double prev_cmd_ax = 0.0;  // 출력 가속도 상태(jerk-제한 rate-limit, mpc_light)

  // ── Phase 1-3: Trapezoidal profile + inline PathController ──
  // Profile uses absolute speed — reverse sign handled at publishCmdVel
  double exit_speed = std::fabs(goal->exit_speed);
  double abs_max_speed = std::fabs(goal->max_linear_speed);
  amr_motion_control::TrapezoidalProfile profile(
    target_distance_, abs_max_speed, goal->acceleration, exit_speed);

  // ── behind-start 가속 옵션 상태 ──
  // accelerate 모드에서 로봇이 시작점 뒤(projection<0)에서 출발하면, 접근 거리(=−projection)
  // 만큼 trapezoidal 프로파일을 앞으로 전치(offset)해 시작점 도달 전 순항속도까지 가속한다.
  // (단순 속도캡만 풀면 projection=0 경계에서 프로파일 속도가 0으로 떨어져 불연속이 생기므로,
  //  거리 전치로 경계 연속성을 보장한다.)
  double approach_offset      = 0.0;    // accelerate 모드에서만 >0
  bool   approach_offset_init = false;  // 첫 iteration 1회 산출
  double accel_ramp_vx        = 0.0;    // accelerate 모드 시간 기반 가속 적분기(Issue 8 fix)

  // Reset localization watchdog for this run
  watchdog_->reset();

  bool reached      = false;
  int tf_fail_count = 0;
  const int tf_fail_max = 50;  // 1 second at 50Hz → abort

  // Track last known path control output for abort reporting
  double last_projection = 0.0;
  double last_e_d        = 0.0;
  double last_e_theta    = 0.0;

  while (rclcpp::ok() && !reached) {
    // ── Read position from /robot_pose (PoseCache, map-frame) ──
    if (lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock())) {
      tf_fail_count = 0;
      watchdog_->updatePose(robot_x, robot_y, robot_yaw);
    } else {
      tf_fail_count++;
      if (tf_fail_count >= tf_fail_max) {
        RCLCPP_ERROR(node_->get_logger(),
          "TF2 lookup failed %d consecutive times at dist=%.3f m",
          tf_fail_count, last_projection);
        finish_abort(-4, last_projection, last_e_d,
                     last_e_theta * 180.0 / M_PI, start_time);
        return;
      }
      // else: use last known position for this cycle
    }

    // Cancel check
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "TranslateReverse cancelled at dist=%.3f m", last_projection);
      finish_abort(-1, last_projection, last_e_d,
                   last_e_theta * 180.0 / M_PI, start_time);
      return;
    }

    // Global timeout check
    if ((node_->now() - start_time).seconds() > max_timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(),
        "TranslateReverse global timeout (%.1f s), dist=%.3f m", max_timeout_sec_, last_projection);
      finish_abort(-3, last_projection, last_e_d,
                   last_e_theta * 180.0 / M_PI, start_time);
      return;
    }

    // Localization watchdog
    if (enable_localization_watchdog_ && !watchdog_->checkHealth()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Localization health check failed at dist=%.3f m", last_projection);
      finish_abort(-4, last_projection, last_e_d,
                   last_e_theta * 180.0 / M_PI, start_time);
      return;
    }

    // ── Compute projection (inline, to avoid double update of PD derivative) ──
    const double rx_p     = robot_x - path_start_x_;
    const double ry_p     = robot_y - path_start_y_;
    const double projection = rx_p * path_ux_ + ry_p * path_uy_;

    // ── behind-start 가속: 시작점 뒤에서 출발 시 접근 거리만큼 프로파일 전치(1회) ──
    if (!approach_offset_init) {
      approach_offset_init = true;
      if (behind_start_accelerate_ && projection < 0.0) {
        approach_offset = -projection;
        profile = amr_motion_control::TrapezoidalProfile(
          target_distance_ + approach_offset, abs_max_speed,
          goal->acceleration, exit_speed);
        RCLCPP_INFO(node_->get_logger(),
          "TranslateReverse behind-start ACCELERATE: approach_offset=%.3f m "
          "(시작점 뒤부터 가속)", approach_offset);
      }
    }

    // Check for endpoint update (velocity continuity)
    if (endpoint_update_pending_.load()) {
      std::lock_guard<std::mutex> lock(endpoint_mutex_);
      double new_start_x = goal->end_x;
      double new_start_y = goal->end_y;
      // Update inline path
      path_start_x_ = new_start_x;
      path_start_y_ = new_start_y;
      path_end_x_   = pending_end_x_;
      path_end_y_   = pending_end_y_;
      theta_path_   = std::atan2(pending_end_y_ - new_start_y,
                                  pending_end_x_ - new_start_x);
      path_ux_ = std::cos(theta_path_);
      path_uy_ = std::sin(theta_path_);
      double new_dist = std::hypot(pending_end_x_ - new_start_x,
                                   pending_end_y_ - new_start_y);
      target_distance_ += new_dist;
      double new_exit_speed = pending_has_next_.load() ? goal->max_linear_speed : 0.0;
      profile = amr_motion_control::TrapezoidalProfile(
        target_distance_, goal->max_linear_speed, goal->acceleration, new_exit_speed);
      approach_offset = 0.0;  // 시작점 통과 후 재기준되므로 전치 해제
      prev_e_theta_ = 0.0;
      prev_omega_   = 0.0;
      e_theta_filter_.reset();
      endpoint_update_pending_.store(false);
      RCLCPP_INFO(node_->get_logger(),
        "Endpoint updated: new target_dist=%.3f m", target_distance_);
    }

    // ── Arrival check ──
    double remaining = target_distance_ - projection;
    // accelerate 모드: 접근 거리만큼 전치된 좌표로 프로파일 조회(시작점 뒤부터 가속).
    // creep 모드: approach_offset=0 → 기존과 동일하게 clamped_projection 사용.
    double clamped_projection = std::max(0.0, projection + approach_offset);
    auto prof_out = profile.getSpeed(clamped_projection);  // phase/feedback 판정용
    double vx_profile = prof_out.speed;
    // ── 시간 기반 가속 v = a·t (accelerate 모드, Issue 8/10 fix) ── (vx_profile 은 양수 크기)
    // 위치 기반 getSpeed(projection)은 projection 을 localized robot_pose 에서 얻는데, 이 pose 는
    // map→base_link(rtabmap 보정 포함)라 값이 jitter(0.1초당 20~150mm 들쭉날쭉)한다. 그 위치를
    // 속도 함수 v=√(2a·s) 에 넣으면 명령속도가 계단(staircase)이 된다. 그래서 가속구간은 위치를 떼고
    // 순수 시간 적분 v += a·dt (= v=a·t) 으로 매끄럽게 만든다. behind_start_kick_(0.1)은 정지마찰
    // 즉시 극복용 floor. 감속은 아래 위치 기반 v_cap 이 담당하므로 정확 정지 정밀도는 유지된다.
    if (behind_start_accelerate_) {
      const double kick = std::min(behind_start_kick_, static_cast<double>(goal->max_linear_speed));
      accel_ramp_vx = std::max(accel_ramp_vx, kick);
      accel_ramp_vx = std::min(accel_ramp_vx + goal->acceleration * dt,
                               static_cast<double>(goal->max_linear_speed));
      vx_profile = accel_ramp_vx;  // 시간 기반 단독 (위치 기반 getSpeed jitter 제거)
    }
    watchdog_->setCurrentSpeed(vx_profile);

    // Behind start: creep 모드에서만 시작점까지 고정 속도로 접근.
    // accelerate 모드에서는 프로파일 전치로 이미 가속 중이므로 캡을 적용하지 않는다.
    if (!behind_start_accelerate_) {
      const double behind_cap =
          std::min(behind_start_speed_, std::fabs(static_cast<double>(goal->max_linear_speed)));
      if (projection < 0.0) {
        vx_profile = behind_cap;
      } else if (prof_out.phase == amr_motion_control::ProfilePhase::ACCEL &&
                 vx_profile < behind_cap) {
        vx_profile = behind_cap;
      }
    }

    // min_vx clamp
    bool near_goal = (remaining < goal_reach_threshold_);
    if (prof_out.phase != amr_motion_control::ProfilePhase::DONE && !near_goal &&
        vx_profile < min_vx_) {
      vx_profile = min_vx_;
    }

    // ── 도착 2단 감속 캡 (T-AMR rosbag 측정 기반, 완전정지 jerk 저감) ──
    // vx_profile 은 절대속도(발행 시 후진 부호 적용). 1단(원거리): v_cap = approach_gain_·remaining,
    // 2단(근거리, v<stop_speed_): v_cap = sqrt(2·stop_decel_·remaining) (도착점 0 수렴).
    // 전환 잔여거리 = stop_speed_^2/(2·stop_decel_). (walk 스무더가 슬루 제한 → 새 불연속 없음)
    // 하이브리드 감속(vx_profile 절대속도): FAR=jerk-제한 S-curve + NEAR=위치 P 피드백(정확 수렴).
    // v_cap = min(sqrt(2·a_decel·remaining), approach_gain·remaining). 위치-P 가 목표 정확 수렴
    // (이전 d_jerk stall 버그 해소), 출력 rate-limit 가 jerk 제한.
    if (projection >= 0.0) {
      const double v_far  = std::sqrt(2.0 * a_decel_max_ * remaining);
      const double v_near = approach_gain_ * remaining;
      const double v_cap = std::min(v_far, v_near);
      if (vx_profile > v_cap) { vx_profile = v_cap; }
    }

    // ── Safety speed limit ──
    double cur_safety_limit = safety_sub_->speed_limit();
    if (std::isfinite(cur_safety_limit)) {
      vx_profile = std::min(vx_profile, cur_safety_limit);
    }
    if (safety_sub_->is_dangerous()) {
      vx_profile = 0.0;
    }

    // Arrival — should_stop 게이트(mpc_light): 위치 도달 + 저속일 때만 확정. blocking ramp 제거 →
    // 정지 중에도 watchdog/safety/cancel 유지. exit_speed==0 시 vx_profile=0 주입 → rate-limit ramp.
    const bool pos_reached = (prof_out.phase == amr_motion_control::ProfilePhase::DONE ||
                              projection >= (target_distance_ - arrival_tolerance_));
    if (pos_reached) {
      if (goal->exit_speed > 0.0) {
        reached = true;
        RCLCPP_INFO(node_->get_logger(),
          "TranslateReverse reached with velocity continuity (exit_speed=%.3f)", goal->exit_speed);
        break;
      }
      if (std::fabs(prev_cmd_vx) < should_stop_speed_) {
        reached = true;
        publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
        break;
      }
      vx_profile = 0.0;
    }

    // ── Inline PathController update ──
    // lateral error (cross product)
    const double rx_c = robot_x - path_start_x_;
    const double ry_c = robot_y - path_start_y_;
    const double e_d  = rx_c * path_uy_ - ry_c * path_ux_;

    // heading error (filtered) — 고속 IMU yaw 를 map-frame 으로 융합; reverse: yaw + 180°
    //   (위치/CTE 는 robot_x/robot_y 만 사용하므로 영향 없음).
    const double robot_yaw_ctrl = fuseImuYawToMap(last_yaw_rad_.load(), imu_yaw_offset);
    const double yaw_for_ctrl = is_reverse_ ? (robot_yaw_ctrl + M_PI) : robot_yaw_ctrl;
    const double e_theta_raw = normalizeAngle(yaw_for_ctrl - theta_path_);
    const double e_theta     = e_theta_filter_.update(e_theta_raw);

    // heading derivative
    double de_theta = 0.0;
    if (dt > 0.0) {
      de_theta = (e_theta - prev_e_theta_) / dt;
    }

    // CTE→omega correction (Stanley-based heading correction)
    double theta_cte = 0.0;
    if (vx_profile > 1e-6) {
      theta_cte = std::atan2(K_stanley_ * e_d, K_soft_ + vx_profile);
    }

    // Heading PD + CTE correction → omega
    double omega = -Kp_heading_ * (e_theta - theta_cte) - Kd_heading_ * de_theta;
    omega = std::clamp(omega, -max_omega_, max_omega_);

    // Angular acceleration limit (smooth omega transitions)
    if (dt > 0.0) {
      double max_omega_change = alpha_max_ * dt;
      double omega_diff = omega - prev_omega_;
      if (std::fabs(omega_diff) > max_omega_change) {
        omega = prev_omega_ + std::copysign(max_omega_change, omega_diff);
      }
    }
    prev_omega_   = omega;
    prev_e_theta_ = e_theta;

    // Track for abort reporting
    last_projection = projection;
    last_e_d        = e_d;
    last_e_theta    = e_theta;

    // ── 출력 가속도 rate-limit (2상태 jerk-제한, mpc_light; vx_profile 절대속도) ──
    {
      const double a_min = -a_decel_max_;
      const double a_max =  walk_accel_limit_;
      double a_des = (vx_profile - prev_cmd_vx) / dt;
      a_des = std::clamp(a_des, a_min, a_max);
      const double j_step = jerk_max_ * dt;
      double a_cmd = std::clamp(a_des, prev_cmd_ax - j_step, prev_cmd_ax + j_step);
      prev_cmd_vx += a_cmd * dt;
      if (vx_profile <= 0.0 && prev_cmd_vx < 0.0) { prev_cmd_vx = 0.0; a_cmd = 0.0; }
      prev_cmd_ax = a_cmd;
    }
    double cmd_vx = prev_cmd_vx;

    // Publish cmd_vel: reverse sign for backward motion
    publishCmdVel(cmd_vel_pub_, is_reverse_ ? -cmd_vx : cmd_vx, omega);

    // Debug log (throttled ~5Hz at 50Hz rate)
    static int dbg_cnt = 0;
    if (++dbg_cnt % 10 == 0) {
      RCLCPP_INFO(node_->get_logger(),
        "[TranslateReverse] vx=%.3f omega=%.4f proj=%.3fm lat=%.3fm hdg=%.1f° e_theta=%.1f° cte=%.1f°",
        is_reverse_ ? -cmd_vx : cmd_vx, omega,
        projection, e_d,
        robot_yaw_ctrl * 180.0 / M_PI,
        e_theta * 180.0 / M_PI,
        theta_cte * 180.0 / M_PI);
    }

    // Feedback
    uint8_t phase_id;
    switch (prof_out.phase) {
      case amr_motion_control::ProfilePhase::ACCEL:  phase_id = 1; break;
      case amr_motion_control::ProfilePhase::CRUISE: phase_id = 2; break;
      case amr_motion_control::ProfilePhase::DECEL:  phase_id = 3; break;
      default:                                        phase_id = 3; break;
    }
    feedback->phase                 = phase_id;
    feedback->current_distance      = projection;
    feedback->current_lateral_error = e_d;
    feedback->current_heading_error = e_theta * 180.0 / M_PI;
    feedback->current_vx            = is_reverse_ ? -cmd_vx : cmd_vx;
    feedback->current_vy            = 0.0;
    feedback->current_omega         = omega;
    feedback->w1_drive_rpm          = 0.0;
    feedback->w2_drive_rpm          = 0.0;
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  // Stop driving
  publishCmdVel(cmd_vel_pub_, 0.0, 0.0);
  watchdog_->setCurrentSpeed(0.0);

  // Read final state from /robot_pose cache
  lookupRobotPose(pose_cache_, robot_x, robot_y, robot_yaw, node_->get_logger(), node_->get_clock());

  // Final path control values
  const double rx_f      = robot_x - path_start_x_;
  const double ry_f      = robot_y - path_start_y_;
  const double final_proj = rx_f * path_ux_ + ry_f * path_uy_;
  const double final_e_d  = rx_f * path_uy_ - ry_f * path_ux_;
  const double yaw_final  = is_reverse_ ? (robot_yaw + M_PI) : robot_yaw;
  const double final_e_th = normalizeAngle(yaw_final - theta_path_);

  RCLCPP_INFO(node_->get_logger(),
    "TranslateReverse complete: dist=%.3f/%.3f m, lat_err=%.4f m, head_err=%.2f deg",
    final_proj, target_distance_, final_e_d, final_e_th * 180.0 / M_PI);

  // Clear path visualization
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = node_->now();
    path_viz_pub_->publish(empty);
  }

  // ── Success ──
  result->status              = 0;
  result->actual_distance     = final_proj;
  result->final_lateral_error = final_e_d;
  result->final_heading_error = final_e_th * 180.0 / M_PI;
  result->elapsed_time        = (node_->now() - start_time).seconds();

  goal_handle->succeed(result);
}

// ════════════════════════════════════════════════════════
//  Endpoint update callback (velocity continuity)
// ════════════════════════════════════════════════════════

void TranslateReverseActionServer::onUpdateEndpoint(
  const std::shared_ptr<amr_interfaces::srv::UpdateTranslateEndpoint::Request> req,
  std::shared_ptr<amr_interfaces::srv::UpdateTranslateEndpoint::Response> res)
{
  std::lock_guard<std::mutex> lock(endpoint_mutex_);
  pending_end_x_ = req->end_x;
  pending_end_y_ = req->end_y;
  pending_has_next_.store(req->has_next);
  endpoint_update_pending_.store(true);
  res->success = true;
  res->message = "Endpoint update queued";
  RCLCPP_INFO(node_->get_logger(), "TranslateReverse endpoint update: (%.3f, %.3f) has_next=%d",
    req->end_x, req->end_y, req->has_next);
}

// ════════════════════════════════════════════════════════
//  Publish / Callbacks
// ════════════════════════════════════════════════════════

void TranslateReverseActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
