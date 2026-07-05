"""Launch the joystick teleop GUI panel."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_gui',
            executable='gui_node',
            name='joystick_gui',
            output='screen',
        ),
    ])
