// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <hdl_graph_slam/msg/floor_coeffs.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>

#include <hdl_graph_slam/srv/save_map.hpp>
#include <hdl_graph_slam/srv/load_graph.hpp>
#include <hdl_graph_slam/srv/dump_graph.hpp>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

#include <pcl_conversions/pcl_conversions.h>

// For UTM conversion (geodesy replacement)
#include <cmath>

namespace hdl_graph_slam {

// Simple UTM conversion (replacing geodesy library)
struct UTMPoint {
  double easting;
  double northing;
  double altitude;
};

// WGS84 ellipsoid parameters
constexpr double WGS84_A = 6378137.0;           // semi-major axis
constexpr double WGS84_E2 = 0.00669437999014;   // first eccentricity squared

inline UTMPoint geoToUTM(double latitude, double longitude, double altitude) {
  UTMPoint utm;

  double lat_rad = latitude * M_PI / 180.0;
  double lon_rad = longitude * M_PI / 180.0;

  // UTM zone
  int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
  double lon_origin = (zone - 1) * 6.0 - 180.0 + 3.0;
  double lon_origin_rad = lon_origin * M_PI / 180.0;

  double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));
  double T = std::tan(lat_rad) * std::tan(lat_rad);
  double C = (WGS84_E2 / (1.0 - WGS84_E2)) * std::cos(lat_rad) * std::cos(lat_rad);
  double A = std::cos(lat_rad) * (lon_rad - lon_origin_rad);

  double e4 = WGS84_E2 * WGS84_E2;
  double e6 = e4 * WGS84_E2;
  double M = WGS84_A * ((1.0 - WGS84_E2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat_rad
                       - (3.0 * WGS84_E2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * std::sin(2.0 * lat_rad)
                       + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat_rad)
                       - (35.0 * e6 / 3072.0) * std::sin(6.0 * lat_rad));

  double k0 = 0.9996;

  utm.easting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0
                         + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * (WGS84_E2 / (1.0 - WGS84_E2))) * A * A * A * A * A / 120.0) + 500000.0;

  utm.northing = k0 * (M + N * std::tan(lat_rad) * (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0
                      + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * (WGS84_E2 / (1.0 - WGS84_E2))) * A * A * A * A * A * A / 720.0));

  if(latitude < 0.0) {
    utm.northing += 10000000.0;
  }

  utm.altitude = altitude;

  return utm;
}

class HdlGraphSlamNode : public rclcpp::Node {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> ApproxSyncPolicy;

  HdlGraphSlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("hdl_graph_slam_node", options) {
    // init parameters
    published_odom_topic_ = this->declare_parameter<std::string>("published_odom_topic", "/odom");
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    map_cloud_resolution_ = this->declare_parameter<double>("map_cloud_resolution", 0.05);
    trans_odom2map_.setIdentity();

    max_keyframes_per_update_ = this->declare_parameter<int>("max_keyframes_per_update", 10);

    //
    anchor_node_ = nullptr;
    anchor_edge_ = nullptr;
    floor_plane_node_ = nullptr;
    initialized_ = false;
    graph_slam_.reset(new GraphSLAM(this->declare_parameter<std::string>("g2o_solver_type", "lm_var")));
    map_cloud_generator_.reset(new MapCloudGenerator());
    nmea_parser_.reset(new NmeaSentenceParser());

    gps_time_offset_ = this->declare_parameter<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy_ = this->declare_parameter<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z_ = this->declare_parameter<double>("gps_edge_stddev_z", 10.0);
    floor_edge_stddev_ = this->declare_parameter<double>("floor_edge_stddev", 10.0);

    imu_time_offset_ = this->declare_parameter<double>("imu_time_offset", 0.0);
    enable_imu_orientation_ = this->declare_parameter<bool>("enable_imu_orientation", false);
    enable_imu_acceleration_ = this->declare_parameter<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev_ = this->declare_parameter<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev_ = this->declare_parameter<double>("imu_acceleration_edge_stddev", 3.0);

    points_topic_ = this->declare_parameter<std::string>("points_topic", "/velodyne_points");

    // Declare parameters that will be used later with get_parameter
    this->declare_parameter<bool>("fix_first_node", false);
    this->declare_parameter<std::string>("fix_first_node_stddev", "1 1 1 1 1 1");
    this->declare_parameter<std::string>("odometry_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("odometry_edge_robust_kernel_size", 1.0);
    this->declare_parameter<std::string>("gps_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("gps_edge_robust_kernel_size", 1.0);
    this->declare_parameter<std::string>("imu_orientation_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("imu_orientation_edge_robust_kernel_size", 1.0);
    this->declare_parameter<std::string>("imu_acceleration_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("imu_acceleration_edge_robust_kernel_size", 1.0);
    this->declare_parameter<std::string>("floor_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("floor_edge_robust_kernel_size", 1.0);
    this->declare_parameter<std::string>("loop_closure_edge_robust_kernel", "NONE");
    this->declare_parameter<double>("loop_closure_edge_robust_kernel_size", 1.0);
    this->declare_parameter<bool>("fix_first_node_adaptive", true);
    this->declare_parameter<int>("g2o_solver_num_iterations", 1024);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // subscribers
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(this, published_odom_topic_));
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, "/filtered_points"));
    sync_.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub_, *cloud_sub_));
    sync_->registerCallback(std::bind(&HdlGraphSlamNode::cloud_callback, this, std::placeholders::_1, std::placeholders::_2));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/gpsimu_driver/imu_data", 1024,
      std::bind(&HdlGraphSlamNode::imu_callback, this, std::placeholders::_1));
    floor_sub_ = this->create_subscription<hdl_graph_slam::msg::FloorCoeffs>(
      "/floor_detection/floor_coeffs", 1024,
      std::bind(&HdlGraphSlamNode::floor_coeffs_callback, this, std::placeholders::_1));

    if(this->declare_parameter<bool>("enable_gps", true)) {
      gps_sub_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
        "/gps/geopoint", 1024,
        std::bind(&HdlGraphSlamNode::gps_callback, this, std::placeholders::_1));
      nmea_sub_ = this->create_subscription<nmea_msgs::msg::Sentence>(
        "/gpsimu_driver/nmea_sentence", 1024,
        std::bind(&HdlGraphSlamNode::nmea_callback, this, std::placeholders::_1));
      navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/navsat", 1024,
        std::bind(&HdlGraphSlamNode::navsat_callback, this, std::placeholders::_1));
    }

    // publishers
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/hdl_graph_slam/map_points", rclcpp::QoS(1).transient_local());
    read_until_pub_ = this->create_publisher<std_msgs::msg::Header>("/hdl_graph_slam/read_until", 32);

    load_service_server_ = this->create_service<hdl_graph_slam::srv::LoadGraph>(
      "/hdl_graph_slam/load",
      std::bind(&HdlGraphSlamNode::load_service, this, std::placeholders::_1, std::placeholders::_2));
    dump_service_server_ = this->create_service<hdl_graph_slam::srv::DumpGraph>(
      "/hdl_graph_slam/dump",
      std::bind(&HdlGraphSlamNode::dump_service, this, std::placeholders::_1, std::placeholders::_2));
    save_map_service_server_ = this->create_service<hdl_graph_slam::srv::SaveMap>(
      "/hdl_graph_slam/save_map",
      std::bind(&HdlGraphSlamNode::save_map_service, this, std::placeholders::_1, std::placeholders::_2));

    graph_updated_ = false;
    double graph_update_interval = this->declare_parameter<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = this->declare_parameter<double>("map_cloud_update_interval", 10.0);
    optimization_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(graph_update_interval),
      std::bind(&HdlGraphSlamNode::optimization_timer_callback, this));
    map_publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(map_cloud_update_interval),
      std::bind(&HdlGraphSlamNode::map_points_publish_timer_callback, this));
  }

private:
  /**
   * @brief Lazy initialization for components that require shared_from_this()
   */
  void initialize_components() {
    if(!initialized_) {
      keyframe_updater_.reset(new KeyframeUpdater(this->shared_from_this()));
      loop_detector_.reset(new LoopDetector(this->shared_from_this()));
      inf_calclator_.reset(new InformationMatrixCalculator(this->shared_from_this()));
      initialized_ = true;
    }
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    // Lazy initialization (shared_from_this not available in constructor)
    initialize_components();

    const rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(base_frame_id_.empty()) {
      base_frame_id_ = cloud_msg->header.frame_id;
    }

    if(!keyframe_updater_->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex_);
      if(keyframe_queue_.empty()) {
        std_msgs::msg::Header read_until;
        read_until.stamp = rclcpp::Time(stamp) + rclcpp::Duration(10, 0);
        read_until.frame_id = points_topic_;
        read_until_pub_->publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub_->publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater_->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex_);
    keyframe_queue_.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex_);

    if(keyframe_queue_.empty()) {
      return false;
    }

    trans_odom2map_mutex_.lock();
    Eigen::Isometry3d odom2map(trans_odom2map_.cast<double>());
    trans_odom2map_mutex_.unlock();

    std::cout << "flush_keyframe_queue - keyframes len:" << keyframes_.size() << std::endl;
    int num_processed = 0;
    for(int i = 0; i < std::min<int>(keyframe_queue_.size(), max_keyframes_per_update_); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue_[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes_.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam_->add_se3_node(odom);
      keyframe_hash_[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes_.empty() && new_keyframes_.size() == 1) {
        bool fix_first_node = false;
        this->get_parameter("fix_first_node", fix_first_node);
        if(fix_first_node) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::string fix_first_node_stddev;
          this->get_parameter("fix_first_node_stddev", fix_first_node_stddev);
          std::stringstream sst(fix_first_node_stddev);
          for(int j = 0; j < 6; j++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(j, j) = 1.0 / stddev;
          }

          anchor_node_ = graph_slam_->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node_->setFixed(true);
          anchor_edge_ = graph_slam_->add_se3_edge(anchor_node_, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      if(i == 0 && keyframes_.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes_.back() : keyframe_queue_[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator_->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);

      std::string odometry_edge_robust_kernel;
      double odometry_edge_robust_kernel_size;
      this->get_parameter("odometry_edge_robust_kernel", odometry_edge_robust_kernel);
      this->get_parameter("odometry_edge_robust_kernel_size", odometry_edge_robust_kernel_size);
      auto edge = graph_slam_->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam_->add_robust_kernel(edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size);
    }

    std_msgs::msg::Header read_until;
    read_until.stamp = rclcpp::Time(keyframe_queue_[num_processed]->stamp) + rclcpp::Duration(10, 0);
    read_until.frame_id = points_topic_;
    read_until_pub_->publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub_->publish(read_until);

    keyframe_queue_.erase(keyframe_queue_.begin(), keyframe_queue_.begin() + num_processed + 1);
    return true;
  }

  /**
   * @brief received NMEA sentences are parsed and converted to gps_callback
   * @param nmea_msg
   */
  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr nmea_msg) {
    GPRMC grmc = nmea_parser_->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  /**
   * @brief received NavSatFix is converted to GeoPointStamped and forwarded to gps_callback
   * @param navsat_msg
   */
  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg) {
    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex_);
    gps_msg->header.stamp = rclcpp::Time(gps_msg->header.stamp) + rclcpp::Duration::from_seconds(gps_time_offset_);
    gps_queue_.push_back(gps_msg);
  }

  /**
   * @brief process GPS queue and add GPS edges to the graph
   * @return if true, at least one GPS edge was added
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex_);

    if(keyframes_.empty() || gps_queue_.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue_.begin();

    for(auto& keyframe : keyframes_) {
      if(keyframe->stamp > rclcpp::Time(gps_queue_.back()->header.stamp)) {
        break;
      }

      if(keyframe->stamp < rclcpp::Time((*gps_cursor)->header.stamp) || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue_.end(); gps++) {
        auto dt = (rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*gps)->header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if(0.2 < std::abs((rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      UTMPoint utm = geoToUTM((*closest_gps)->position.latitude, (*closest_gps)->position.longitude, (*closest_gps)->position.altitude);
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

      // the first gps data position will be the origin of the map
      if(!zero_utm_) {
        zero_utm_ = xyz;
      }
      xyz -= (*zero_utm_);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(xyz.z())) {
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy_;
        edge = graph_slam_->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy_;
        information_matrix(2, 2) /= gps_edge_stddev_z_;
        edge = graph_slam_->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }

      std::string gps_edge_robust_kernel;
      double gps_edge_robust_kernel_size;
      this->get_parameter("gps_edge_robust_kernel", gps_edge_robust_kernel);
      this->get_parameter("gps_edge_robust_kernel_size", gps_edge_robust_kernel_size);
      graph_slam_->add_robust_kernel(edge, gps_edge_robust_kernel, gps_edge_robust_kernel_size);

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue_.begin(), gps_queue_.end(), keyframes_.back()->stamp,
      [](const rclcpp::Time& stamp, const geographic_msgs::msg::GeoPointStamped::SharedPtr& geopoint) {
        return stamp < rclcpp::Time(geopoint->header.stamp);
      });
    gps_queue_.erase(gps_queue_.begin(), remove_loc);
    return updated;
  }

  /**
   * @brief received IMU data is added to #imu_queue
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    if(!enable_imu_orientation_ && !enable_imu_acceleration_) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex_);
    imu_msg->header.stamp = rclcpp::Time(imu_msg->header.stamp) + rclcpp::Duration::from_seconds(imu_time_offset_);
    imu_queue_.push_back(imu_msg);
  }

  /**
   * @brief process IMU queue and add IMU edges to the graph
   * @return if true, at least one IMU edge was added
   */
  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex_);
    if(keyframes_.empty() || imu_queue_.empty() || base_frame_id_.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue_.begin();

    for(auto& keyframe : keyframes_) {
      if(keyframe->stamp > rclcpp::Time(imu_queue_.back()->header.stamp)) {
        break;
      }

      if(keyframe->stamp < rclcpp::Time((*imu_cursor)->header.stamp) || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue_.end(); imu++) {
        auto dt = (rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*imu)->header.stamp) - keyframe->stamp).seconds();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs((rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp).seconds())) {
        continue;
      }

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::msg::Vector3Stamped acc_imu;
      geometry_msgs::msg::Vector3Stamped acc_base;
      geometry_msgs::msg::QuaternionStamped quat_imu;
      geometry_msgs::msg::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = rclcpp::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;

      try {
        tf_buffer_->transform(acc_imu, acc_base, base_frame_id_, tf2::durationFromSec(0.0));
        tf_buffer_->transform(quat_imu, quat_base, base_frame_id_, tf2::durationFromSec(0.0));
      } catch(std::exception& e) {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }

      keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
      keyframe->orientation = keyframe->orientation;
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if(enable_imu_orientation_) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev_;
        auto edge = graph_slam_->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);

        std::string imu_orientation_edge_robust_kernel;
        double imu_orientation_edge_robust_kernel_size;
        this->get_parameter("imu_orientation_edge_robust_kernel", imu_orientation_edge_robust_kernel);
        this->get_parameter("imu_orientation_edge_robust_kernel_size", imu_orientation_edge_robust_kernel_size);
        graph_slam_->add_robust_kernel(edge, imu_orientation_edge_robust_kernel, imu_orientation_edge_robust_kernel_size);
      }

      if(enable_imu_acceleration_) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev_;
        g2o::OptimizableGraph::Edge* edge = graph_slam_->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);

        std::string imu_acceleration_edge_robust_kernel;
        double imu_acceleration_edge_robust_kernel_size;
        this->get_parameter("imu_acceleration_edge_robust_kernel", imu_acceleration_edge_robust_kernel);
        this->get_parameter("imu_acceleration_edge_robust_kernel_size", imu_acceleration_edge_robust_kernel_size);
        graph_slam_->add_robust_kernel(edge, imu_acceleration_edge_robust_kernel, imu_acceleration_edge_robust_kernel_size);
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue_.begin(), imu_queue_.end(), keyframes_.back()->stamp,
      [](const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::SharedPtr& imu) {
        return stamp < rclcpp::Time(imu->header.stamp);
      });
    imu_queue_.erase(imu_queue_.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam::msg::FloorCoeffs::SharedPtr floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex_);
    floor_coeffs_queue_.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex_);

    if(keyframes_.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes_.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue_) {
      if(rclcpp::Time(floor_coeffs->header.stamp) > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash_.find(rclcpp::Time(floor_coeffs->header.stamp));
      if(found == keyframe_hash_.end()) {
        continue;
      }

      if(!floor_plane_node_) {
        floor_plane_node_ = graph_slam_->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node_->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev_);

      std::string floor_edge_robust_kernel;
      double floor_edge_robust_kernel_size;
      this->get_parameter("floor_edge_robust_kernel", floor_edge_robust_kernel);
      this->get_parameter("floor_edge_robust_kernel_size", floor_edge_robust_kernel_size);
      auto edge = graph_slam_->add_se3_plane_edge(keyframe->node, floor_plane_node_, coeffs, information);
      graph_slam_->add_robust_kernel(edge, floor_edge_robust_kernel, floor_edge_robust_kernel_size);

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue_.begin(), floor_coeffs_queue_.end(), latest_keyframe_stamp,
      [](const rclcpp::Time& stamp, const hdl_graph_slam::msg::FloorCoeffs::SharedPtr& coeffs) {
        return stamp < rclcpp::Time(coeffs->header.stamp);
      });
    floor_coeffs_queue_.erase(floor_coeffs_queue_.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   */
  void map_points_publish_timer_callback() {
    if(map_points_pub_->get_subscription_count() == 0 || !graph_updated_) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex_.lock();
    snapshot = keyframes_snapshot_;
    keyframes_snapshot_mutex_.unlock();

    auto cloud = map_cloud_generator_->generate(snapshot, map_cloud_resolution_);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id_;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);

    map_points_pub_->publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   */
  void optimization_timer_callback() {
    std::lock_guard<std::mutex> lock(main_thread_mutex_);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::msg::Header read_until;
      read_until.stamp = this->now() + rclcpp::Duration(30, 0);
      read_until.frame_id = points_topic_;
      read_until_pub_->publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub_->publish(read_until);
    }

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector_->detect(keyframes_, new_keyframes_, *graph_slam_);
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator_->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);

      std::string loop_closure_edge_robust_kernel;
      double loop_closure_edge_robust_kernel_size;
      this->get_parameter("loop_closure_edge_robust_kernel", loop_closure_edge_robust_kernel);
      this->get_parameter("loop_closure_edge_robust_kernel_size", loop_closure_edge_robust_kernel_size);
      auto edge = graph_slam_->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam_->add_robust_kernel(edge, loop_closure_edge_robust_kernel, loop_closure_edge_robust_kernel_size);
    }

    std::copy(new_keyframes_.begin(), new_keyframes_.end(), std::back_inserter(keyframes_));
    new_keyframes_.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    bool fix_first_node_adaptive = true;
    this->get_parameter("fix_first_node_adaptive", fix_first_node_adaptive);
    if(anchor_node_ && fix_first_node_adaptive) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge_->vertices()[1])->estimate();
      anchor_node_->setEstimate(anchor_target);
    }

    // optimize the pose graph
    int num_iterations = 1024;
    this->get_parameter("g2o_solver_num_iterations", num_iterations);
    graph_slam_->optimize(num_iterations);

    // publish tf
    const auto& keyframe = keyframes_.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex_.lock();
    trans_odom2map_ = trans.matrix().cast<float>();
    trans_odom2map_mutex_.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes_.size());
    std::transform(keyframes_.begin(), keyframes_.end(), snapshot.begin(), [](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex_.lock();
    keyframes_snapshot_.swap(snapshot);
    keyframes_snapshot_mutex_.unlock();
    graph_updated_ = true;

    if(odom2map_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id_, odom_frame_id_);
      odom2map_pub_->publish(ts);
    }

    if(markers_pub_->get_subscription_count() > 0) {
      auto markers = create_marker_array(this->now());
      markers_pub_->publish(markers);
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::msg::MarkerArray create_marker_array(const rclcpp::Time& stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.resize(4);

    // node markers
    visualization_msgs::msg::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::msg::Marker& imu_marker = markers.markers[1];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
    imu_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes_.size());
    traj_marker.colors.resize(keyframes_.size());
    for(size_t i = 0; i < keyframes_.size(); i++) {
      Eigen::Vector3d pos = keyframes_[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes_.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes_[i]->acceleration) {
        Eigen::Vector3d pos = keyframes_[i]->node->estimate().translation();
        geometry_msgs::msg::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::msg::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam_->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam_->graph->edges().size() * 2);

    auto edge_itr = graph_slam_->graph->edges().begin();
    for(size_t i = 0; edge_itr != graph_slam_->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam_->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam_->graph->vertices().size();
        edge_marker.colors[i * 2].r = 1.0 - p1;
        edge_marker.colors[i * 2].g = p1;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
        edge_marker.colors[i * 2 + 1].g = p2;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i * 2].z += 0.5;
          edge_marker.points[i * 2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].b = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].b = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    visualization_msgs::msg::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;

    if(!keyframes_.empty()) {
      Eigen::Vector3d pos = keyframes_.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z();
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector_->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;

    return markers;
  }

  /**
   * @brief load all data from a directory
   * @param req
   * @param res
   */
  void load_service(const std::shared_ptr<hdl_graph_slam::srv::LoadGraph::Request> req,
                    std::shared_ptr<hdl_graph_slam::srv::LoadGraph::Response> res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex_);

    std::string directory = req->path;

    std::cout << "loading data from:" << directory << std::endl;

    // Load graph.
    graph_slam_->load(directory + "/graph.g2o");

    // Iterate over the items in this directory and count how many sub directories there are.
    // This will give an upper limit on how many keyframe indexes we can expect to find.
    boost::filesystem::directory_iterator begin(directory), end;
    int max_directory_count = std::count_if(begin, end,
      [](const boost::filesystem::directory_entry& d) {
        return boost::filesystem::is_directory(d.path()); // only return true if a directory
      });

    // Load keyframes by looping through key frame indexes that we expect to see.
    for(int i = 0; i < max_directory_count; i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;
      std::string key_frame_directory = sst.str();

      // If key_frame_directory doesnt exist, then we have run out so lets stop looking.
      if(!boost::filesystem::is_directory(key_frame_directory)) {
        break;
      }

      KeyFrame::Ptr keyframe(new KeyFrame(key_frame_directory, graph_slam_->graph.get()));
      keyframes_.push_back(keyframe);
    }
    std::cout << "loaded " << keyframes_.size() << " keyframes" << std::endl;

    // Load special nodes.
    std::ifstream ifs(directory + "/special_nodes.csv");
    if(!ifs) {
      res->success = false;
      return;
    }
    while(!ifs.eof()) {
      std::string token;
      ifs >> token;
      if(token == "anchor_node") {
        int id = 0;
        ifs >> id;
        anchor_node_ = static_cast<g2o::VertexSE3*>(graph_slam_->graph->vertex(id));
      } else if(token == "anchor_edge") {
        int id = 0;
        ifs >> id;
        // We have no way of directly pulling the edge based on the edge ID that we have just read in.
        // Fortunately anchor edges are always attached to the anchor node so we can iterate over
        // the edges that listed against the node until we find the one that matches our ID.
        if(anchor_node_) {
          auto edges = anchor_node_->edges();

          for(auto e : edges) {
            int edgeID = e->id();
            if(edgeID == id) {
              anchor_edge_ = static_cast<g2o::EdgeSE3*>(e);
              break;
            }
          }
        }
      } else if(token == "floor_node") {
        int id = 0;
        ifs >> id;
        floor_plane_node_ = static_cast<g2o::VertexPlane*>(graph_slam_->graph->vertex(id));
      }
    }

    // check if we have any non null special nodes, if all are null then lets not bother.
    if(anchor_node_ || anchor_edge_ || floor_plane_node_) {
      std::cout << "loaded special nodes -";

      // check exists before printing information about each special node
      if(anchor_node_) {
        std::cout << " anchor_node: " << anchor_node_->id();
      }
      if(anchor_edge_) {
        std::cout << " anchor_edge: " << anchor_edge_->id();
      }
      if(floor_plane_node_) {
        std::cout << " floor_node: " << floor_plane_node_->id();
      }

      // finish with a new line
      std::cout << std::endl;
    }

    // Update our keyframe snapshot so we can publish a map update, trigger update with graph_updated = true.
    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes_.size());

    std::transform(keyframes_.begin(), keyframes_.end(), snapshot.begin(), [](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex_.lock();
    keyframes_snapshot_.swap(snapshot);
    keyframes_snapshot_mutex_.unlock();
    graph_updated_ = true;

    res->success = true;

    std::cout << "snapshot updated" << std::endl << "loading successful" << std::endl;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   */
  void dump_service(const std::shared_ptr<hdl_graph_slam::srv::DumpGraph::Request> req,
                    std::shared_ptr<hdl_graph_slam::srv::DumpGraph::Response> res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex_);

    std::string directory = req->destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "dumping data to:" << directory << std::endl;
    // save graph
    graph_slam_->save(directory + "/graph.g2o");

    // save keyframes
    for(size_t i = 0; i < keyframes_.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes_[i]->save(sst.str());
    }

    if(zero_utm_) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm_->x() % zero_utm_->y() % zero_utm_->z() << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node_ == nullptr ? -1 : anchor_node_->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge_ == nullptr ? -1 : anchor_edge_->id()) << std::endl;
    ofs << "floor_node " << (floor_plane_node_ == nullptr ? -1 : floor_plane_node_->id()) << std::endl;

    res->success = true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   */
  void save_map_service(const std::shared_ptr<hdl_graph_slam::srv::SaveMap::Request> req,
                        std::shared_ptr<hdl_graph_slam::srv::SaveMap::Response> res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex_.lock();
    snapshot = keyframes_snapshot_;
    keyframes_snapshot_mutex_.unlock();

    auto cloud = map_cloud_generator_->generate(snapshot, req->resolution);
    if(!cloud) {
      res->success = false;
      return;
    }

    if(zero_utm_ && req->utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm_).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id_;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm_) {
      std::ofstream ofs(req->destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm_->x() % zero_utm_->y() % zero_utm_->z() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req->destination, *cloud);
    res->success = ret == 0;
  }

private:
  // ROS
  rclcpp::TimerBase::SharedPtr optimization_timer_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<hdl_graph_slam::msg::FloorCoeffs>::SharedPtr floor_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  std::string published_odom_topic_;
  std::string map_frame_id_;
  std::string odom_frame_id_;

  std::mutex trans_odom2map_mutex_;
  Eigen::Matrix4f trans_odom2map_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub_;

  std::string points_topic_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<hdl_graph_slam::srv::LoadGraph>::SharedPtr load_service_server_;
  rclcpp::Service<hdl_graph_slam::srv::DumpGraph>::SharedPtr dump_service_server_;
  rclcpp::Service<hdl_graph_slam::srv::SaveMap>::SharedPtr save_map_service_server_;

  // keyframe queue
  std::string base_frame_id_;
  std::mutex keyframe_queue_mutex_;
  std::deque<KeyFrame::Ptr> keyframe_queue_;

  // gps queue
  double gps_time_offset_;
  double gps_edge_stddev_xy_;
  double gps_edge_stddev_z_;
  boost::optional<Eigen::Vector3d> zero_utm_;
  std::mutex gps_queue_mutex_;
  std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue_;

  // imu queue
  double imu_time_offset_;
  bool enable_imu_orientation_;
  double imu_orientation_edge_stddev_;
  bool enable_imu_acceleration_;
  double imu_acceleration_edge_stddev_;
  std::mutex imu_queue_mutex_;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;

  // floor_coeffs queue
  double floor_edge_stddev_;
  std::mutex floor_coeffs_queue_mutex_;
  std::deque<hdl_graph_slam::msg::FloorCoeffs::SharedPtr> floor_coeffs_queue_;

  // for lazy initialization
  bool initialized_;

  // for map cloud generation
  std::atomic_bool graph_updated_;
  double map_cloud_resolution_;
  std::mutex keyframes_snapshot_mutex_;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot_;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator_;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex_;

  int max_keyframes_per_update_;
  std::deque<KeyFrame::Ptr> new_keyframes_;

  g2o::VertexSE3* anchor_node_;
  g2o::EdgeSE3* anchor_edge_;
  g2o::VertexPlane* floor_plane_node_;
  std::vector<KeyFrame::Ptr> keyframes_;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash_;

  std::unique_ptr<GraphSLAM> graph_slam_;
  std::unique_ptr<LoopDetector> loop_detector_;
  std::unique_ptr<KeyframeUpdater> keyframe_updater_;
  std::unique_ptr<NmeaSentenceParser> nmea_parser_;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator_;
};

}  // namespace hdl_graph_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_graph_slam::HdlGraphSlamNode)
