"""
slam.launch.py  (v3 - WSL2 + GPU version)
------------------------------------------
Replaces the VirtualBox slow-scan workaround with a proper
lifecycle-managed startup. slam_toolbox on ROS Jazzy is a
lifecycle node so we must configure → activate it explicitly.

Where to put this file:
    ~/ros2_ws/src/amr_description/launch/slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    slam_params = os.path.join(
        get_package_share_directory('amr_description'),
        'config',
        'slam_toolbox_params.yaml'
    )

    # Start slam_toolbox node (will be in 'unconfigured' state)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True},
        ],
    )

    # After 5 seconds, configure the lifecycle node
    configure_slam = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen',
            )
        ]
    )

    # After 10 seconds, activate the lifecycle node
    activate_slam = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        slam_node,
        configure_slam,
        activate_slam,
    ])
