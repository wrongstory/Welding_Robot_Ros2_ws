#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tc_msgs/msg/safety_status.hpp>

#include <atomic>
#include <limits>
#include <mutex>
#include <string>

namespace amr_safety_core
{

struct SafetySubscriberConfig
{
  std::string safety_status_topic = "safety_status";
  std::string speed_limit_topic   = "/safety/speed_limit";
  std::string estop_topic         = "/estop";
  std::string robot_pose_topic    = "/robot_pose";  // map-frame pose SSOT
  std::string map_frame           = "map";          // (deprecated: TF pose 미사용)
  std::string base_frame          = "base_link";    // (deprecated: TF pose 미사용)
  double safety_status_timeout    = 0.5;
  double pose_timeout             = 1.0;
};

// Template class: supports both rclcpp::Node and rclcpp_lifecycle::LifecycleNode
template<typename NodeT>
class SafetySubscriberT
{
public:
  explicit SafetySubscriberT(NodeT * node,
                             const SafetySubscriberConfig & cfg = {})
  : node_(node), cfg_(cfg),
    safety_status_(tc_msgs::msg::SafetyStatus::STATUS_NORMAL),
    speed_limit_(std::numeric_limits<double>::infinity()),
    estop_pressed_(false),
    safety_enabled_(true)
  {
    // /safety/enable: UI에서 safety on/off 제어
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable();
      qos.transient_local();
      safety_enable_sub_ = node_->template create_subscription<std_msgs::msg::Bool>(
        "/safety/enable", qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          safety_enabled_.store(msg->data);
        });
    }

    // safety_status: Reliable KeepLast(10) deadline
    {
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.reliable();
      qos.deadline(rclcpp::Duration::from_seconds(cfg_.safety_status_timeout));
      safety_status_sub_ = node_->template create_subscription<tc_msgs::msg::SafetyStatus>(
        cfg_.safety_status_topic, qos,
        [this](const tc_msgs::msg::SafetyStatus::SharedPtr msg) {
          safety_status_.store(msg->status);
          std::lock_guard<std::mutex> lk(time_mutex_);
          last_safety_time_ = node_->now();
        });
    }

    // /safety/speed_limit: Reliable KeepLast(1) transient_local
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable();
      qos.transient_local();
      speed_limit_sub_ = node_->template create_subscription<std_msgs::msg::Float64>(
        cfg_.speed_limit_topic, qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          speed_limit_.store(msg->data);
        });
    }

    // /estop: Reliable KeepLast(1) transient_local
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.reliable();
      qos.transient_local();
      estop_sub_ = node_->template create_subscription<std_msgs::msg::Bool>(
        cfg_.estop_topic, qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          estop_pressed_.store(msg->data);
        });
    }

    // /robot_pose: map-frame robot pose (PoseStamped, RELIABLE KeepLast(10))
    // tr_works_pose_publisher 발행자 QoS = RELIABLE depth=10 에 매칭.
    {
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.reliable();
      robot_pose_sub_ = node_->template create_subscription<geometry_msgs::msg::PoseStamped>(
        cfg_.robot_pose_topic, qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(pose_mutex_);
          last_pose_ = *msg;
          last_pose_time_ = node_->now();
          pose_received_.store(true);
        });
    }
  }

  uint8_t safety_status() const { return safety_status_.load(); }

  bool is_dangerous() const
  {
    if (!safety_enabled_.load()) return false;
    return safety_status_.load() == tc_msgs::msg::SafetyStatus::STATUS_DANGEROUS;
  }

  bool is_warning() const
  {
    return safety_status_.load() == tc_msgs::msg::SafetyStatus::STATUS_WARNING;
  }

  double speed_limit() const { return speed_limit_.load(); }

  // /robot_pose 기반 pose 유효성 검사 (수신 시각 staleness)
  bool pose_valid() const
  {
    if (!safety_enabled_.load()) return true;
    if (!pose_received_.load()) return false;
    std::lock_guard<std::mutex> lk(pose_mutex_);
    double age = (node_->now() - last_pose_time_).seconds();
    return age <= cfg_.pose_timeout;
  }

  bool estop_pressed() const { return estop_pressed_.load(); }

  bool safety_status_fresh() const
  {
    std::lock_guard<std::mutex> lk(time_mutex_);
    if (!last_safety_time_.nanoseconds()) {
      return false;
    }
    double elapsed = (node_->now() - last_safety_time_).seconds();
    return elapsed <= cfg_.safety_status_timeout;
  }

  // /robot_pose 기반 현재 pose 조회 (캐시 반환)
  geometry_msgs::msg::PoseStamped last_pose() const
  {
    std::lock_guard<std::mutex> lk(pose_mutex_);
    return last_pose_;
  }

  void set_pose_timeout(double sec)
  {
    cfg_.pose_timeout = sec;
  }

private:
  NodeT * node_;
  SafetySubscriberConfig cfg_;

  std::atomic<uint8_t> safety_status_;
  std::atomic<double>  speed_limit_;
  std::atomic<bool>    estop_pressed_;
  std::atomic<bool>    safety_enabled_;

  mutable std::mutex time_mutex_;
  rclcpp::Time last_safety_time_{0, 0, RCL_ROS_TIME};

  // /robot_pose 캐시 (map-frame pose SSOT, TF lookup 대체)
  mutable std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> pose_received_{false};
  typename rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;

  typename rclcpp::Subscription<tc_msgs::msg::SafetyStatus>::SharedPtr safety_status_sub_;
  typename rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     speed_limit_sub_;
  typename rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        estop_sub_;
  typename rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        safety_enable_sub_;
};

// Backward-compatible alias
using SafetySubscriber = SafetySubscriberT<rclcpp::Node>;

}  // namespace amr_safety_core
