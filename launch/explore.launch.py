"""
explore.launch.py
-----------------
Launches frontier-based autonomous exploration for amr_description.

Uses: frontier_exploration_ros2 (mertgulerx/frontier_exploration_ros2)
      Executable: frontier_explorer
      Params:     config/frontier_explorer_params.yaml  (in amr_description)

Prerequisites — ALL must be running before starting this file:
  Terminal 1:  ros2 launch amr_description spawn.launch.py
  Terminal 2:  ros2 launch amr_description slam.launch.py
  Terminal 3:  ros2 launch amr_description nav2.launch.py use_slam:=true
  Terminal 4:  ros2 launch amr_description explore.launch.py   ← this file

The explorer reads /map (SLAM Toolbox) and /global_costmap/costmap (Nav2),
finds frontier cells, and sends NavigateToPose goals to Nav2 automatically
until the entire map is explored.

Save the finished map:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Our custom params file — tuned to this robot
    amr_pkg = get_package_share_directory('amr_description')
    our_params = os.path.join(amr_pkg, 'config', 'frontier_explorer_params.yaml')

    # Use the package's own launch file — it handles all the argument
    # wiring correctly (namespace, log_level, QoS overrides, etc.)
    frontier_pkg_share = get_package_share_directory('frontier_exploration_ros2')
    frontier_launch = os.path.join(
        frontier_pkg_share, 'launch', 'frontier_explorer.launch.py'
    )

    explorer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frontier_launch),
        launch_arguments={
            # Point to our tuned params file
            'params_file':   our_params,
            # Must be true — we are in Gazebo simulation
            'use_sim_time':  'true',
            # Start exploring immediately on node activation
            'autostart':     'true',
            # Map is published as transient_local by SLAM Toolbox
            'map_qos_durability': 'transient_local',
            # Verbose logging during first runs — change to 'info' later
            'log_level':     'info',
        }.items(),
    )

    # Delay 15 s so Nav2 (which has its own 5-s internal delay) has
    # fully activated all lifecycle nodes before the first goal is sent.
    # If you still see "Waiting for action server", increase this value.
    delayed_explorer = TimerAction(
        period=15.0,
        actions=[explorer],
    )

    return LaunchDescription([
        delayed_explorer,
    ])