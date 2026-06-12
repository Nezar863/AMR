"""
teleop.launch.py
----------------
Launches keyboard teleop remapped to diff_drive_controller.
Uses stamped:=true because the controller expects TwistStamped.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel'),
        ],
        parameters=[{
            'stamped': True,
        }],
    )

    return LaunchDescription([teleop])