import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_motion_control_2wd')

    config_file = os.path.join(pkg_share, 'config', 'motion_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),

        DeclareLaunchArgument(
            'config_file', default_value=config_file,
            description='Path to motion control config YAML'),

        # cmd_vel_safety_guard 동시 실행 여부. ACS 의 Robot Base(robot_base.launch.py)가
        # 이미 guard 를 띄우면 중복(=/cmd_vel 이중 publisher) 되므로 GUI Motion Start 는
        # launch_guard:=false 로 호출한다. 단독/HIL 자율 실행 호환을 위해 기본값 true.
        DeclareLaunchArgument(
            'launch_guard', default_value='true',
            description='Also launch cmd_vel_safety_guard. Set false when a base '
                        'bringup (ACS robot_base) already provides the guard.'),

        # IMU 는 iAHRS 고정(imu/data). 모션 heading 제어가 IMU orientation(쿼터니언)을
        # 요구하는데 Livox 내장 IMU 는 raw 만 발행(orientation 미발행)하여 부적합.
        # 따라서 imu_topic 선택/remap 없이 imu/data(iAHRS) 단일 사용.
        Node(
            package='amr_motion_control_2wd',
            executable='amr_motion_control_2wd_node',
            name='amr_motion_control_2wd',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('cmd_vel', 'cmd_vel_raw')],
        ),

        # Safety gate: /cmd_vel_raw (motion 출력) → clamp/estop/watchdog → /cmd_vel.
        # diff_drive_controller 가 /cmd_vel 을 구독하므로 이 노드가 체인을 잇는다.
        # 체인: motion → /cmd_vel_raw → cmd_vel_safety_guard → /cmd_vel → diff_drive_controller
        Node(
            package='cmd_vel_safety_guard',
            executable='cmd_vel_safety_guard_node',
            name='cmd_vel_safety_guard',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_guard')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel',
                'guarded_topic': '/cmd_vel_guarded',
                'watchdog_timeout_s': 0.3,
            }],
        ),
    ])
