#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace amr_motion_control_2wd
{

// ── 액션 상호 배제 ──
// 단일 프로세스 내 모든 액션 서버가 공유하는 전역 atomic.
// handle_goal에서 CAS acquire, ActionGuard 소멸자에서 자동 release.
enum class ActiveAction : uint8_t {
  NONE = 0, SPIN, TURN, TRANSLATE, TRANSLATE_REVERSE, YAW_CONTROL, PURE_PURSUIT
};

extern std::atomic<ActiveAction> g_active_action;

inline const char* to_string(ActiveAction a)
{
  switch (a) {
    case ActiveAction::NONE:              return "NONE";
    case ActiveAction::SPIN:              return "SPIN";
    case ActiveAction::TURN:              return "TURN";
    case ActiveAction::TRANSLATE:         return "TRANSLATE";
    case ActiveAction::TRANSLATE_REVERSE: return "TRANSLATE_REVERSE";
    case ActiveAction::YAW_CONTROL:       return "YAW_CONTROL";
    case ActiveAction::PURE_PURSUIT:      return "PURE_PURSUIT";
    default:                              return "UNKNOWN";
  }
}

// ── RAII Guard: execute() 종료 시 자동 해제 ──
// handle_goal에서 CAS acquire → execute() 시작 시 ActionGuard 생성
// execute() 종료 (정상/예외/abort/cancel) 시 소멸자가 자동 NONE 복구
class ActionGuard
{
public:
  ActionGuard() = default;
  ~ActionGuard() { g_active_action.store(ActiveAction::NONE); }
  ActionGuard(const ActionGuard &) = delete;
  ActionGuard & operator=(const ActionGuard &) = delete;
};

// ── 공통 유틸리티 (Phase 2) ──

// 파라미터 안전 선언/읽기: 이미 선언된 파라미터는 get, 아니면 declare
template <typename T>
inline T safeParam(rclcpp::Node::SharedPtr node,
                   const std::string & name, T default_val)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_val);
  }
  return node->get_parameter(name).get_value<T>();
}

// cmd_vel 발행 (publisher는 각 서버가 소유)
inline void publishCmdVel(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
    double linear_x, double angular_z)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  pub->publish(msg);
}

// 각도 정규화 (radian 입출력, [-PI, PI])
inline double normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

// ── PoseCache: /robot_pose 구독 + 신선도 판정 ──
// 기존 TF2 map->base_link lookup을 대체. /robot_pose (PoseStamped, frame map,
// 50Hz, tr_works_pose_publisher 발행) 를 구독해 최신 pose를 캐시한다.
// 발행자 QoS = RELIABLE depth=10 이므로 구독자도 동일하게 매칭한다.
// action execute()는 detached thread에서 돌므로 mutex로 보호.
class PoseCache
{
public:
  PoseCache(rclcpp::Node * node,
            const std::string & topic = "/robot_pose",
            double staleness_threshold = 0.2)
  : node_(node),
    clock_(node->get_clock())
  {
    if (!node_->has_parameter("pose_staleness_threshold")) {
      node_->declare_parameter("pose_staleness_threshold", staleness_threshold);
    }
    staleness_threshold_ =
      node_->get_parameter("pose_staleness_threshold").get_value<double>();

    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_pose_ = *msg;
        last_recv_ = clock_->now();
        received_ = true;
      });
  }

  // 신선한 pose가 있으면 x,y,yaw(rad) 채우고 true. 없거나 stale면 false.
  bool getPose(double & x, double & y, double & yaw) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isFreshLocked()) { return false; }
    x = last_pose_.pose.position.x;
    y = last_pose_.pose.position.y;
    yaw = yawFromQuat(last_pose_.pose.orientation);
    return true;
  }

  // spin용: yaw(rad)만. 동일한 신선도 규칙.
  bool getYaw(double & yaw) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isFreshLocked()) { return false; }
    yaw = yawFromQuat(last_pose_.pose.orientation);
    return true;
  }

  bool isFresh() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return isFreshLocked();
  }

private:
  bool isFreshLocked() const
  {
    if (!received_) { return false; }
    double age = (clock_->now() - last_recv_).seconds();
    return age <= staleness_threshold_;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  rclcpp::Node * node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  double staleness_threshold_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_recv_;
  bool received_{false};
};

// map→base_link pose 읽기 (PoseCache 기반). 반환 의미는 기존과 동일:
// pose 획득 실패(=stale 또는 미수신) 시 false, 콜러의 기존 실패 경로가 동작.
inline bool lookupRobotPose(
    const PoseCache & pose_cache,
    double & x, double & y, double & yaw,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock)
{
  if (pose_cache.getPose(x, y, yaw)) {
    return true;
  }
  RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
    "/robot_pose stale or not received");
  return false;
}

// map→base_link yaw만 읽기 (degrees). 반환 의미 기존과 동일.
inline bool lookupTfYaw(
    const PoseCache & pose_cache,
    double & yaw_deg,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock)
{
  double yaw_rad;
  if (!pose_cache.getYaw(yaw_rad)) {
    RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
      "/robot_pose stale or not received");
    return false;
  }
  yaw_deg = yaw_rad * 180.0 / M_PI;
  return true;
}

// IMU yaw delta wrap+deadband 필터링
// 반환값: degrees (signed). 0.0 = deadband 미만.
// prev_yaw_rad: deadband 초과 시에만 갱신됨 (소음 누적 방지)
inline double readImuDelta(
    double current_yaw_rad, double & prev_yaw_rad,
    double deadband_rad)
{
  double delta = current_yaw_rad - prev_yaw_rad;
  if (delta >  M_PI) { delta -= 2.0 * M_PI; }
  if (delta < -M_PI) { delta += 2.0 * M_PI; }
  if (std::abs(delta) < deadband_rad) { return 0.0; }
  prev_yaw_rad = current_yaw_rad;
  return delta * 180.0 / M_PI;
}

// ── IMU yaw → map-frame yaw 융합 (직진/heading 프리미티브용) ──
// /robot_pose yaw 는 map-frame 권위값이지만 본 프로젝트에선 느림(~1Hz, rtabmap localization).
// IMU yaw(/imu/data) 는 고속(50~200Hz)이나 절대 0 점이 임의 프레임이다.
// 시작 시 1회 오프셋 offset = normalizeAngle(pose_yaw0 - imu_yaw0) 을 등록(imuYawOffset)하고,
// 매 제어주기 map-frame yaw = normalizeAngle(imu_yaw + offset) 로 환산(fuseImuYawToMap)해
// heading 피드백을 고속화한다. 위치(projection)·cross-track error 는 /robot_pose 를 그대로 유지.
// 느린 IMU yaw 드리프트는 위치기반 Stanley CTE 항이 흡수하므로 seed-once 로 충분하다
// (spin/turn 의 IMU 상대제어와 동일 계열; reference trnav 표준은 pose-yaw 만 써 ~1Hz pose 환경에서
//  50Hz heading 추종이 부정확했음).
inline double imuYawOffset(double pose_yaw_rad, double imu_yaw_rad)
{
  return normalizeAngle(pose_yaw_rad - imu_yaw_rad);
}

inline double fuseImuYawToMap(double imu_yaw_rad, double yaw_offset_rad)
{
  return normalizeAngle(imu_yaw_rad + yaw_offset_rad);
}

}  // namespace amr_motion_control_2wd
