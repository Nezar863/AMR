"""
ekf.launch.py
-------------
Starts the robot_localization EKF node that fuses
wheel odometry + IMU into a clean /odometry/filtered topic
and publishes the corrected odom → base_link TF.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ekf_params = os.path.join(
        get_package_share_directory('amr_description'),
        'config',
        'ekf_params.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ekf_node])