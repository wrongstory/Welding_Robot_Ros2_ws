// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/msg/scan_matching_status.hpp>

namespace hdl_graph_slam {

class ScanMatchingOdometryNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("scan_matching_odometry_node", options) {
    RCLCPP_DEBUG(this->get_logger(), "initializing scan_matching_odometry_node...");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    keyframe_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    initialize_params();

    if(this->declare_parameter<bool>("enable_imu_frontend", false)) {
      msf_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/msf_core/pose", 1,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          this->msf_pose_callback(msg, false);
        });
      msf_pose_after_update_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/msf_core/pose_after_update", 1,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          this->msf_pose_callback(msg, true);
        });
    }

    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/filtered_points", 256,
      std::bind(&ScanMatchingOdometryNode::cloud_callback, this, std::placeholders::_1));
    read_until_pub_ = this->create_publisher<std_msgs::msg::Header>("/scan_matching_odometry/read_until", 32);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(published_odom_topic_, 32);
    trans_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/scan_matching_odometry/transform", 32);
    status_pub_ = this->create_publisher<hdl_graph_slam::msg::ScanMatchingStatus>("/scan_matching_odometry/status", 8);
    aligned_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 32);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    published_odom_topic_ = this->declare_parameter<std::string>("published_odom_topic", "/odom");
    points_topic_ = this->declare_parameter<std::string>("points_topic", "/velodyne_points");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    robot_odom_frame_id_ = this->declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans_ = this->declare_parameter<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle_ = this->declare_parameter<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time_ = this->declare_parameter<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    transform_thresholding_ = this->declare_parameter<bool>("transform_thresholding", false);
    max_acceptable_trans_ = this->declare_parameter<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle_ = this->declare_parameter<double>("max_acceptable_angle", 1.0);

    // enable robot odometry for initial guess
    this->declare_parameter<bool>("enable_robot_odometry_init_guess", false);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = this->declare_parameter<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter_.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter_ = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter_ = passthrough;
    }

    // Defer registration initialization to first callback (shared_from_this not available in constructor)
    registration_ = nullptr;
  }

  void initialize_registration() {
    if(!registration_) {
      registration_ = select_registration_method(this->shared_from_this());
    }
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    if(!rclcpp::ok()) {
      return;
    }

    // Lazy initialization of registration (shared_from_this not available in constructor)
    initialize_registration();

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Matrix4f pose = matching(stamp, cloud);
    publish_odometry(stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::msg::Header read_until;
    read_until.frame_id = points_topic_;
    read_until.stamp = rclcpp::Time(cloud_msg->header.stamp) + rclcpp::Duration(1, 0);
    read_until_pub_->publish(read_until);

    read_until.frame_id = "/filtered_points";
    read_until_pub_->publish(read_until);
  }

  void msf_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg, bool after_update) {
    if(after_update) {
      msf_pose_after_update_ = pose_msg;
    } else {
      msf_pose_ = pose_msg;
    }
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter_) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter_->setInputCloud(cloud);
    downsample_filter_->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4f matching(const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe_) {
      prev_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      prev_trans_.setIdentity();
      keyframe_pose_.setIdentity();
      keyframe_stamp_ = stamp;
      keyframe_ = downsample(cloud);
      registration_->setInputTarget(keyframe_);
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    registration_->setInputSource(filtered);

    std::string msf_source;
    Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

    bool enable_imu_frontend = false;
    this->get_parameter("enable_imu_frontend", enable_imu_frontend);
    if(enable_imu_frontend) {
      if(msf_pose_ && rclcpp::Time(msf_pose_->header.stamp) > keyframe_stamp_ &&
         msf_pose_after_update_ && rclcpp::Time(msf_pose_after_update_->header.stamp) > keyframe_stamp_) {
        Eigen::Isometry3d pose0 = pose2isometry(msf_pose_after_update_->pose.pose);
        Eigen::Isometry3d pose1 = pose2isometry(msf_pose_->pose.pose);
        Eigen::Isometry3d delta = pose0.inverse() * pose1;

        msf_source = "imu";
        msf_delta = delta.cast<float>();
      } else {
        std::cerr << "msf data is too old" << std::endl;
      }
    } else if(this->get_parameter("enable_robot_odometry_init_guess").as_bool() && prev_time_.nanoseconds() != 0) {
      geometry_msgs::msg::TransformStamped transform;
      rclcpp::Time time_zero(0, 0, RCL_ROS_TIME);
      if(tf_buffer_->canTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time_, robot_odom_frame_id_, rclcpp::Duration(0, 0))) {
        transform = tf_buffer_->lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time_, robot_odom_frame_id_, rclcpp::Duration(0, 0));
      } else if(tf_buffer_->canTransform(cloud->header.frame_id, time_zero, cloud->header.frame_id, prev_time_, robot_odom_frame_id_, rclcpp::Duration(0, 0))) {
        transform = tf_buffer_->lookupTransform(cloud->header.frame_id, time_zero, cloud->header.frame_id, prev_time_, robot_odom_frame_id_, rclcpp::Duration(0, 0));
      }

      if(rclcpp::Time(transform.header.stamp).nanoseconds() == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id_);
      } else {
        msf_source = "odometry";
        msf_delta = tf2isometry(transform).cast<float>();
      }
    }

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration_->align(*aligned, prev_trans_ * msf_delta.matrix());

    publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    if(!registration_->hasConverged()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "scan matching has not converged!!");
      RCLCPP_INFO_STREAM(this->get_logger(), "ignore this frame(" << stamp.nanoseconds() << ")");
      return keyframe_pose_ * prev_trans_;
    }

    Eigen::Matrix4f trans = registration_->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose_ * trans;

    if(transform_thresholding_) {
      Eigen::Matrix4f delta = prev_trans_.inverse() * trans;
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

      if(dx > max_acceptable_trans_ || da > max_acceptable_angle_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "too large transform!!  " << dx << "[m] " << da << "[rad]");
        RCLCPP_INFO_STREAM(this->get_logger(), "ignore this frame(" << stamp.nanoseconds() << ")");
        return keyframe_pose_ * prev_trans_;
      }
    }

    prev_time_ = stamp;
    prev_trans_ = trans;

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose_, odom_frame_id_, "keyframe");
    keyframe_broadcaster_->sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp_).seconds();
    if(delta_trans > keyframe_delta_trans_ || delta_angle > keyframe_delta_angle_ || delta_time > keyframe_delta_time_) {
      keyframe_ = filtered;
      registration_->setInputTarget(keyframe_);

      keyframe_pose_ = odom;
      keyframe_stamp_ = stamp;
      prev_time_ = stamp;
      prev_trans_.setIdentity();
    }

    if(aligned_points_pub_->get_subscription_count() > 0) {
      pcl::transformPointCloud(*cloud, *aligned, odom);
      aligned->header.frame_id = odom_frame_id_;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_points_pub_->publish(aligned_msg);
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const rclcpp::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // publish transform stamped for IMU integration
    geometry_msgs::msg::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id_, base_frame_id);
    trans_pub_->publish(odom_trans);

    // broadcast the transform over tf
    odom_broadcaster_->sendTransform(odom_trans);

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);
  }

  /**
   * @brief publish scan matching status
   */
  void publish_scan_matching_status(const rclcpp::Time& stamp, const std::string& frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source, const Eigen::Isometry3f& msf_delta) {
    if(status_pub_->get_subscription_count() == 0) {
      return;
    }

    hdl_graph_slam::msg::ScanMatchingStatus status;
    status.header.frame_id = frame_id;
    status.header.stamp = stamp;
    status.has_converged = registration_->hasConverged();
    status.matching_error = registration_->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(size_t i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    status.relative_pose = isometry2pose(Eigen::Isometry3f(registration_->getFinalTransformation()).cast<double>());

    if(!msf_source.empty()) {
      status.prediction_labels.resize(1);
      status.prediction_labels[0].data = msf_source;

      status.prediction_errors.resize(1);
      Eigen::Isometry3f error = Eigen::Isometry3f(registration_->getFinalTransformation()).inverse() * msf_delta;
      status.prediction_errors[0] = isometry2pose(error.cast<double>());
    }

    status_pub_->publish(status);
  }

private:
  // ROS topics
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr msf_pose_after_update_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr trans_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_pub_;
  rclcpp::Publisher<hdl_graph_slam::msg::ScanMatchingStatus>::SharedPtr status_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> keyframe_broadcaster_;

  std::string published_odom_topic_;
  std::string points_topic_;
  std::string odom_frame_id_;
  std::string robot_odom_frame_id_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub_;

  // keyframe parameters
  double keyframe_delta_trans_;  // minimum distance between keyframes
  double keyframe_delta_angle_;  //
  double keyframe_delta_time_;   //

  // registration validation by thresholding
  bool transform_thresholding_;  //
  double max_acceptable_trans_;  //
  double max_acceptable_angle_;

  // odometry calculation
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msf_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msf_pose_after_update_;

  rclcpp::Time prev_time_;
  Eigen::Matrix4f prev_trans_;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose_;               // keyframe pose
  rclcpp::Time keyframe_stamp_;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe_;  // keyframe point cloud

  //
  pcl::Filter<PointT>::Ptr downsample_filter_;
  pcl::Registration<PointT, PointT>::Ptr registration_;
};

}  // namespace hdl_graph_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_graph_slam::ScanMatchingOdometryNode)
