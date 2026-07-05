#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # src 폴더 경로 (rviz config 수정/저장 편의를 위해)
    src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    # RViz config - src 폴더에서 직접 로드 (수정/저장 편의)
    rviz_config = os.path.join(src_dir, 'rviz2', 'hdl_graph_slam.rviz')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')
    enable_floor_detection_arg = DeclareLaunchArgument(
        'enable_floor_detection', default_value='false',
        description='Enable floor detection')
    enable_gps_arg = DeclareLaunchArgument(
        'enable_gps', default_value='false',
        description='Enable GPS')
    enable_imu_acc_arg = DeclareLaunchArgument(
        'enable_imu_acc', default_value='false',
        description='Enable IMU acceleration')
    enable_imu_ori_arg = DeclareLaunchArgument(
        'enable_imu_ori', default_value='false',
        description='Enable IMU orientation')

    # Frame arguments
    points_topic_arg = DeclareLaunchArgument(
        'points_topic', default_value='/livox/lidar',
        description='Topic for pointcloud points')
    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id', default_value='map',
        description='Map frame ID')
    base_link_frame_id_arg = DeclareLaunchArgument(
        'base_link_frame_id', default_value='base_link',
        description='Base link frame ID')
    lidar_odom_frame_id_arg = DeclareLaunchArgument(
        'lidar_odom_frame_id', default_value='odom',
        description='Lidar odometry frame ID')
    published_odom_topic_arg = DeclareLaunchArgument(
        'published_odom_topic', default_value='/odom',
        description='Published odometry topic')

    # Optional arguments
    enable_robot_odometry_init_guess_arg = DeclareLaunchArgument(
        'enable_robot_odometry_init_guess', default_value='false',
        description='Enable robot odometry init guess')
    robot_odom_frame_id_arg = DeclareLaunchArgument(
        'robot_odom_frame_id', default_value='robot_odom',
        description='Robot odometry frame ID')
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id', default_value='livox_frame',
        description='Lidar frame ID')

    # Static transform: base_link to livox_frame (from LIO-SAM config)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox',
        arguments='0.05 0 0.42 0 0 0 base_link livox_frame'.split(' '),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Prefiltering node
    prefiltering_node = Node(
        package='hdl_graph_slam',
        executable='prefiltering_node',
        name='prefiltering_node',
        output='screen',
        remappings=[
            ('/velodyne_points', LaunchConfiguration('points_topic'))
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_link_frame': LaunchConfiguration('base_link_frame_id'),
            'use_distance_filter': False,
            'distance_near_thresh': 0.1,
            'distance_far_thresh': 1000.0,
            'downsample_method': 'VOXELGRID',
            'downsample_resolution': 0.1,
            'outlier_removal_method': 'NONE',
            'statistical_mean_k': 30,
            'statistical_stddev': 1.2,
            'radius_radius': 0.5,
            'radius_min_neighbors': 2,
        }]
    )

    # Scan matching odometry node
    scan_matching_odometry_node = Node(
        package='hdl_graph_slam',
        executable='scan_matching_odometry_node',
        name='scan_matching_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'points_topic': LaunchConfiguration('points_topic'),
            'published_odom_topic': LaunchConfiguration('published_odom_topic'),
            'odom_frame_id': LaunchConfiguration('lidar_odom_frame_id'),
            'robot_odom_frame_id': LaunchConfiguration('robot_odom_frame_id'),
            'keyframe_delta_trans': 1.0,
            'keyframe_delta_angle': 1.0,
            'keyframe_delta_time': 10000.0,
            'transform_thresholding': False,
            'enable_robot_odometry_init_guess': LaunchConfiguration('enable_robot_odometry_init_guess'),
            'max_acceptable_trans': 1.0,
            'max_acceptable_angle': 1.0,
            'downsample_method': 'NONE',
            'downsample_resolution': 0.1,
            'registration_method': 'FAST_GICP',
            'reg_num_threads': 0,
            'reg_transformation_epsilon': 0.01,
            'reg_maximum_iterations': 64,
            'reg_max_correspondence_distance': 2.5,
            'reg_max_optimizer_iterations': 20,
            'reg_use_reciprocal_correspondences': False,
            'reg_correspondence_randomness': 20,
            'reg_resolution': 1.0,
            'reg_nn_search_method': 'DIRECT7',
        }]
    )

    # Floor detection node (conditional)
    floor_detection_node = Node(
        package='hdl_graph_slam',
        executable='floor_detection_node',
        name='floor_detection_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_floor_detection')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'points_topic': LaunchConfiguration('points_topic'),
            'tilt_deg': 0.0,
            'sensor_height': 2.0,
            'height_clip_range': 1.0,
            'floor_pts_thresh': 512,
            'use_normal_filtering': True,
            'normal_filter_thresh': 20.0,
        }]
    )

    # HDL Graph SLAM node
    hdl_graph_slam_node = Node(
        package='hdl_graph_slam',
        executable='hdl_graph_slam_node',
        name='hdl_graph_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'points_topic': LaunchConfiguration('points_topic'),
            'published_odom_topic': LaunchConfiguration('published_odom_topic'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'odom_frame_id': LaunchConfiguration('lidar_odom_frame_id'),
            # Optimization params
            'g2o_solver_type': 'lm_var_cholmod',
            'g2o_solver_num_iterations': 512,
            # Constraint switches
            'enable_gps': LaunchConfiguration('enable_gps'),
            'enable_imu_acceleration': LaunchConfiguration('enable_imu_acc'),
            'enable_imu_orientation': LaunchConfiguration('enable_imu_ori'),
            # Keyframe registration params
            'max_keyframes_per_update': 10,
            'keyframe_delta_trans': 2.0,
            'keyframe_delta_angle': 2.0,
            # Fix first node
            'fix_first_node': True,
            'fix_first_node_stddev': '10 10 10 1 1 1',
            'fix_first_node_adaptive': True,
            # Loop closure params
            'distance_thresh': 20.0,
            'accum_distance_thresh': 35.0,
            'min_edge_interval': 5.0,
            'fitness_score_thresh': 0.5,
            # Scan matching params
            'registration_method': 'FAST_GICP',
            'reg_num_threads': 4,
            'reg_transformation_epsilon': 0.01,
            'reg_maximum_iterations': 64,
            'reg_max_correspondence_distance': 2.5,
            'reg_max_optimizer_iterations': 20,
            'reg_use_reciprocal_correspondences': False,
            'reg_correspondence_randomness': 20,
            'reg_resolution': 1.0,
            'reg_nn_search_method': 'DIRECT7',
            # Edge params - GPS
            'gps_edge_robust_kernel': 'NONE',
            'gps_edge_robust_kernel_size': 1.0,
            'gps_edge_stddev_xy': 20.0,
            'gps_edge_stddev_z': 5.0,
            # Edge params - IMU orientation
            'imu_orientation_edge_robust_kernel': 'NONE',
            'imu_orientation_edge_stddev': 1.0,
            # Edge params - IMU acceleration
            'imu_acceleration_edge_robust_kernel': 'NONE',
            'imu_acceleration_edge_stddev': 1.0,
            # Edge params - floor
            'floor_edge_robust_kernel': 'NONE',
            'floor_edge_stddev': 10.0,
            # Edge params - odometry
            'odometry_edge_robust_kernel': 'NONE',
            'odometry_edge_robust_kernel_size': 1.0,
            'loop_closure_edge_robust_kernel': 'Huber',
            'loop_closure_edge_robust_kernel_size': 1.0,
            # Information matrix params
            'use_const_inf_matrix': False,
            'const_stddev_x': 0.5,
            'const_stddev_q': 0.1,
            'var_gain_a': 20.0,
            'min_stddev_x': 0.1,
            'max_stddev_x': 5.0,
            'min_stddev_q': 0.05,
            'max_stddev_q': 0.2,
            # Update params
            'graph_update_interval': 3.0,
            'map_cloud_update_interval': 10.0,
            'map_cloud_resolution': 0.05,
        }]
    )

    # Map to Odom TF publisher (Python node)
    map2odom_publisher_node = Node(
        package='hdl_graph_slam',
        executable='map2odom_publisher.py',
        name='map2odom_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'odom_frame_id': LaunchConfiguration('lidar_odom_frame_id'),
        }]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_floor_detection_arg,
        enable_gps_arg,
        enable_imu_acc_arg,
        enable_imu_ori_arg,
        points_topic_arg,
        map_frame_id_arg,
        base_link_frame_id_arg,
        lidar_odom_frame_id_arg,
        published_odom_topic_arg,
        enable_robot_odometry_init_guess_arg,
        robot_odom_frame_id_arg,
        lidar_frame_id_arg,
        # Nodes
        static_tf_node,
        prefiltering_node,
        scan_matching_odometry_node,
        floor_detection_node,
        hdl_graph_slam_node,
        map2odom_publisher_node,
        rviz_node,
    ])
