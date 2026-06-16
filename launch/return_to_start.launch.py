"""
return_to_start.launch.py
--------------------------
Launches return_to_start_node.py for the amr_description package.

Run AFTER all four existing terminals are already up:
  Terminal 1: ros2 launch amr_description spawn.launch.py
  Terminal 2: ros2 launch amr_description slam.launch.py
  Terminal 3: ros2 launch amr_description nav2.launch.py use_slam:=true
  Terminal 4: ros2 launch amr_description explore.launch.py
  Terminal 5: ros2 launch amr_description return_to_start.launch.py  ← this file

The node can be started at any point BEFORE exploration finishes.
It subscribes to `exploration_complete` with transient_local QoS, so it
will not miss the event even if it joins slightly late — but it MUST be
started before Nav2 and SLAM are shut down, so starting it here alongside
explore.launch.py is the safest approach.

Optional launch argument:
  nav_server_timeout_sec — how long to wait for the navigate_to_pose
                           action server (default: 15.0 s).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('amr_description')
    params_file = os.path.join(pkg, 'config', 'return_to_start_params.yaml')

    nav_timeout_arg = DeclareLaunchArgument(
        'nav_server_timeout_sec',
        default_value='15.0',
        description='Seconds to wait for the navigate_to_pose action server.',
    )

    return_node = Node(
        package='amr_description',
        executable='return_to_start_node.py',
        name='return_to_start_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'use_sim_time': True,
                'nav_server_timeout_sec': LaunchConfiguration('nav_server_timeout_sec'),
            },
        ],
    )

    return LaunchDescription([
        nav_timeout_arg,
        return_node,
    ])