"""
nav2.launch.py
--------------
Launches the Nav2 autonomous navigation stack for amr_description.

TWO MODES — select with the use_slam argument:

  EXPLORATION MODE  (default)  use_slam:=true
    SLAM Toolbox is already running (slam.launch.py).
    Nav2 uses the live /map from SLAM for planning.
    map_server and amcl are NOT started.
    frontier explorer (explore.launch.py) sends goals automatically.

  KNOWN-MAP MODE               use_slam:=false  map:=/path/to/map.yaml
    A previously saved map is loaded by map_server.
    AMCL localizes the robot in that map.
    Send goals via RViz '2D Nav Goal' or the action client.

Jazzy-specific changes applied:
  - nav2_behaviors  (was nav2_recoveries in older distros)
  - enable_stamped_cmd_vel: True  (TwistStamped throughout)
  - behavior_server  (was recoveries_server)
  - velocity_smoother and smoother_server added

Run AFTER:  ros2 launch amr_description spawn.launch.py
Then:       ros2 launch amr_description slam.launch.py   (exploration mode)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('amr_description')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    use_slam = LaunchConfiguration('use_slam')
    map_yaml  = LaunchConfiguration('map')

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description=(
            'true  = exploration mode: SLAM Toolbox provides live /map, '
            'no map_server or amcl started. '
            'false = known-map mode: load saved map + AMCL localization.'
        ),
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to saved map YAML file (only used when use_slam:=false).',
    )

    # ------------------------------------------------------------------ #
    # Core Nav2 nodes — run in BOTH modes
    # ------------------------------------------------------------------ #

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
    )

    # cmd_vel remapped so Nav2 drives through the existing diff_drive_controller.
    # enable_stamped_cmd_vel is set inside nav2_params.yaml but we also pass it
    # here explicitly so it is guaranteed even if the yaml is misconfigured.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': True, 'enable_stamped_cmd_vel': True},
        ],
        remappings=[
            # Nav2 publishes to /cmd_vel — remap to your controller's topic
            ('cmd_vel', '/diff_drive_controller/cmd_vel'),
        ],
    )

    # Jazzy name: behavior_server  (not recoveries_server)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': True, 'enable_stamped_cmd_vel': True},
        ],
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel'),
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': True, 'enable_stamped_cmd_vel': True},
        ],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': True, 'enable_stamped_cmd_vel': True},
        ],
        remappings=[
            ('cmd_vel',        '/diff_drive_controller/cmd_vel'),
            ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel'),
        ],
    )

    # Lifecycle manager for navigation nodes (both modes)
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'smoother_server',
                'velocity_smoother',
                'bt_navigator',
            ],
        }],
    )

    # ------------------------------------------------------------------ #
    # KNOWN-MAP MODE nodes  (only when use_slam:=false)
    # ------------------------------------------------------------------ #

    map_server = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': True, 'yaml_filename': map_yaml},
        ],
    )

    amcl = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
    )

    # Separate lifecycle manager for localization nodes (known-map mode only)
    lifecycle_manager_loc = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ------------------------------------------------------------------ #
    # Delay navigation stack by 5 s to let SLAM / Gazebo fully settle
    # ------------------------------------------------------------------ #
    delayed_nav = TimerAction(
        period=15.0,
        actions=[
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            smoother_server,
            velocity_smoother,
            lifecycle_manager_nav,
            # Known-map localization (conditionally started)
            map_server,
            amcl,
            lifecycle_manager_loc,
        ],
    )

    return LaunchDescription([
        declare_use_slam,
        declare_map,
        delayed_nav,
    ])