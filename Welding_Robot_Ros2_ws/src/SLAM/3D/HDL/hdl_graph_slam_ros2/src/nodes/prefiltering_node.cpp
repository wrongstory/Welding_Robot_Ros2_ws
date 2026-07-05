// SPDX-License-Identifier: BSD-2-Clause

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hdl_graph_slam {

class PrefilteringNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("prefiltering_node", options) {
    initialize_params();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if(this->declare_parameter<bool>("deskewing", false)) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 1,
        std::bind(&PrefilteringNode::imu_callback, this, std::placeholders::_1));
    }

    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::SensorDataQoS().keep_last(64),
      std::bind(&PrefilteringNode::cloud_callback, this, std::placeholders::_1));
    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 32);
    colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_points", 32);
  }

private:
  void initialize_params() {
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
    }

    std::string outlier_removal_method = this->declare_parameter<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = this->declare_parameter<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = this->declare_parameter<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter_ = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = this->declare_parameter<double>("radius_radius", 0.8);
      int min_neighbors = this->declare_parameter<int>("radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter_ = rad;
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter_ = this->declare_parameter<bool>("use_distance_filter", true);
    distance_near_thresh_ = this->declare_parameter<double>("distance_near_thresh", 1.0);
    distance_far_thresh_ = this->declare_parameter<double>("distance_far_thresh", 100.0);

    base_link_frame_ = this->declare_parameter<std::string>("base_link_frame", "");
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    imu_queue_.push_back(imu_msg);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PointCloud<PointT>::Ptr src_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *src_cloud);

    if(src_cloud->empty()) {
      return;
    }

    pcl::PointCloud<PointT>::ConstPtr cloud = deskewing(src_cloud);

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!base_link_frame_.empty()) {
      if(!tf_buffer_->canTransform(base_link_frame_, cloud->header.frame_id, tf2::TimePointZero)) {
        std::cerr << "failed to find transform between " << base_link_frame_ << " and " << cloud->header.frame_id << std::endl;
      }

      try {
        // Wait for transform with timeout (same as ROS1: ros::Duration(2.0))
        auto transform = tf_buffer_->lookupTransform(
          base_link_frame_, cloud->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(2.0));

        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform.transform);
        pcl::transformPointCloud(*cloud, *transformed, eigen_transform.cast<float>());
        transformed->header.frame_id = base_link_frame_;
        transformed->header.stamp = cloud->header.stamp;
        cloud = transformed;
      } catch(tf2::TransformException& ex) {
        // ROS1 does not have exception handling here, but ROS2 requires try-catch
        // Continue with original cloud (same behavior as ROS1 when transform fails)
        RCLCPP_WARN(this->get_logger(), "Failed to find transform: %s", ex.what());
      }
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    points_pub_->publish(output);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter_) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter_->setInputCloud(cloud);
    downsample_filter_->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter_) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter_->setInputCloud(cloud);
    outlier_removal_filter_->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      return d > distance_near_thresh_ && d < distance_far_thresh_;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    rclcpp::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    if(imu_queue_.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if(colored_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for(size_t i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }
      sensor_msgs::msg::PointCloud2 colored_msg;
      pcl::toROSMsg(*colored, colored_msg);
      colored_pub_->publish(colored_msg);
    }

    sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_queue_.front();

    auto loc = imu_queue_.begin();
    for(; loc != imu_queue_.end(); loc++) {
      imu_msg = (*loc);
      if(rclcpp::Time((*loc)->header.stamp) > stamp) {
        break;
      }
    }

    imu_queue_.erase(imu_queue_.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    double scan_period = this->get_parameter_or<double>("scan_period", 0.1);
    for(size_t i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_link_frame_;

  bool use_distance_filter_;
  double distance_near_thresh_;
  double distance_far_thresh_;

  pcl::Filter<PointT>::Ptr downsample_filter_;
  pcl::Filter<PointT>::Ptr outlier_removal_filter_;
};

}  // namespace hdl_graph_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_graph_slam::PrefilteringNode)
