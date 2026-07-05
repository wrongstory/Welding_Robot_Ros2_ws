import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # src 폴더 경로 (rviz config 수정/저장 편의를 위해)
    # launch 파일: .../hdl_localization/launch/hdl_localization.launch.py
    # rviz 파일:   .../hdl_localization/rviz2/hdl_localization.rviz
    src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    rviz_config = os.path.join(src_dir, 'rviz2', 'hdl_localization.rviz')

    # Launch arguments
    points_topic = LaunchConfiguration('points_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id')
    use_imu = LaunchConfiguration('use_imu')
    use_global_localization = LaunchConfiguration('use_global_localization')
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')
    downsample_resolution = LaunchConfiguration('downsample_resolution')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'points_topic',
            default_value='/livox/lidar',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/imu',
            description='Input IMU topic'
        ),
        DeclareLaunchArgument(
            'odom_child_frame_id',
            default_value='livox_frame',
            description='Odometry child frame ID'
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Use IMU for pose estimation'
        ),
        DeclareLaunchArgument(
            'use_global_localization',
            default_value='false',
            description='Use global localization'
        ),
        DeclareLaunchArgument(
            'globalmap_pcd',
            default_value=os.path.expanduser('~/parking_robot_ros2_ws/map/hdl_slam/KAIST_hdl_slam/map.pcd'),
            description='Path to global map PCD file'
        ),
        DeclareLaunchArgument(
            'downsample_resolution',
            default_value='0.05',
            description='Downsample resolution for global map (larger value = faster but less accurate)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        # Static transform: base_link to livox_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox',
            arguments=['0.05', '0', '0.42', '0', '0', '0', 'base_link', 'livox_frame'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Global map server node
        Node(
            package='hdl_localization',
            executable='hdl_localization_map_server',
            name='globalmap_server',
            output='screen',
            parameters=[{
                'globalmap_pcd': globalmap_pcd,
                'convert_utm_to_local': False,
                'downsample_resolution': downsample_resolution,
                'use_sim_time': use_sim_time,
            }]
        ),

        # HDL localization node
        Node(
            package='hdl_localization',
            executable='hdl_localization_node',
            name='hdl_localization',
            output='screen',
            parameters=[{
                # Simulation time
                'use_sim_time': use_sim_time,
                # Frame settings
                'odom_child_frame_id': odom_child_frame_id,
                # IMU settings
                'use_imu': use_imu,
                'invert_acc': False,
                'invert_gyro': False,
                'cool_time_duration': 2.0,
                # Robot odometry prediction
                'enable_robot_odometry_prediction': False,
                'robot_odom_frame_id': 'odom',
                # NDT settings
                'reg_method': 'NDT_OMP',
                'ndt_neighbor_search_method': 'DIRECT7',
                'ndt_neighbor_search_radius': 2.0,
                'ndt_resolution': 1.0,
                'downsample_resolution': 0.05,
                # Initial pose (set to origin, use RViz 2D Pose Estimate to set)
                'specify_init_pose': False,
                'init_pos_x': 0.0,
                'init_pos_y': 0.0,
                'init_pos_z': 0.0,
                'init_ori_w': 1.0,
                'init_ori_x': 0.0,
                'init_ori_y': 0.0,
                'init_ori_z': 0.0,
                # Global localization
                'use_global_localization': use_global_localization,
            }],
            remappings=[
                ('/velodyne_points', points_topic),
                ('/gpsimu_driver/imu_data', imu_topic),
                ('/odom', '/hdl/odom'),
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz)
        ),
    ])
