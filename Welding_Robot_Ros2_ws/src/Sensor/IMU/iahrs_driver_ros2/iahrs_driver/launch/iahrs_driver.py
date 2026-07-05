#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    iahrs_driver_node = Node(
        package='iahrs_driver',
        executable='iahrs_driver',
        output='screen',
        # /dev/IMU(ttyUSB0) 가 직전 인스턴스/타 프로세스에 일시 점유되면 iahrs 가
        # "Failed to open" → FATAL 종료하여 /imu/data 가 영영 안 나온다(= cartographer
        # use_imu_data 시 "waiting for imu"→스캔 안나옴의 실제 원인). respawn 으로 2초마다
        # 재시도 → 포트가 풀리면 자동 복구.
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {"serial_port": "/dev/IMU"},
            # m_bSingle_TF_option=False: disable the driver's dynamic base_link->imu_link
            # TF (which loads IMU orientation as rotation); 3D SLAM IMU integration uses a
            # static mount TF instead (a dynamic transform would be time-varying/inaccurate).
            {"m_bSingle_TF_option": False},
            # Unused while m_bSingle_TF_option=False; kept consistent with the real mount (0.58 m).
            {"tf_translation_z": 0.58}
        ]
    )

    # base_link -> imu_link static mount TF.
    # Physical mount position of the IMU on this robot: (0, 0, 0.58) m.
    # (2D SLAM / Cartographer is NOT used in this project, so the former
    #  colocation-at-zero constraint from the original FITO project does not apply;
    #  use the real mount height.)
    # identity rotation: iAHRS mounted aligned with base_link.
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0.58',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        output='screen',
    )

    # create and return launch description object
    return LaunchDescription(
        [
            iahrs_driver_node,
            static_tf_base_to_imu
        ]
    )
