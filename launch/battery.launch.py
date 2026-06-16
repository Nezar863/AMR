"""
battery.launch.py
------------------
Launches battery_manager_node with config/battery_config.yaml.

Run AFTER all four existing terminals are up:
  Terminal 1: ros2 launch amr_description spawn.launch.py
  Terminal 2: ros2 launch amr_description slam.launch.py
  Terminal 3: ros2 launch amr_description nav2.launch.py use_slam:=true
  Terminal 4: ros2 launch amr_description explore.launch.py
  Terminal 5: ros2 launch amr_description battery.launch.py   <- this file

This node can be started at any point before or after exploration
completes — it subscribes to a transient_local completion topic, so it
will not miss the event even if started late. Starting it alongside the
other terminals (e.g. right after explore.launch.py) is recommended so
that battery drain tracking covers the whole exploration run.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    battery_params = os.path.join(
        get_package_share_directory('amr_description'),
        'config',
        'battery_config.yaml',
    )

    battery_node = Node(
        package='amr_description',
        executable='battery_manager_node.py',
        name='battery_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            battery_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([battery_node])