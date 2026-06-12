"""
display.launch.py
-----------------
Launches robot_state_publisher + joint_state_publisher_gui + RViz.
Use this to visualise the URDF without Gazebo.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('amr_description')

    # --- xacro → URDF string ------------------------------------------------
    urdf_path = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    robot_description = subprocess.check_output(
        ['xacro', urdf_path]
    ).decode()

    # --- RViz config --------------------------------------------------------
    rviz_config = os.path.join(pkg, 'rviz', 'amr.rviz')

    # --- Launch argument to optionally disable GUI --------------------------
    use_gui = LaunchConfiguration('use_gui', default='true')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui if true'
        ),

        # Publishes /tf and /tf_static from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': False}]
        ),

        # GUI slider to manually move joints (wheels) for visualisation
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
