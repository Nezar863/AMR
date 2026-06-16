"""
spawn.launch.py  (updated for Nav2 autonomy)
--------------------------------------------
Main simulation launch file for AMR in Gazebo Harmonic.
Starts: Gazebo → robot_state_publisher → spawn robot →
        controllers → topic bridges → EKF → RViz

Changes from the original teleop version:
  1. RViz now loads slam_rviz.rviz by default (shows /map, /scan,
     costmaps, and the planned path — better for autonomous operation).
     Change rviz_config back to amr.rviz if you prefer the old view.
  2. No other changes — teleop still works if you run teleop.launch.py.
     Nav2 simply replaces teleop by publishing to the same cmd_vel topic.
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)


def generate_launch_description():

    pkg = get_package_share_directory('amr_description')

    install_description_dir_path = get_package_prefix('amr_description') + '/share'

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        if install_description_dir_path not in os.environ['GZ_SIM_RESOURCE_PATH']:
            os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_description_dir_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(install_description_dir_path)

    # Add Fuel cache so Gazebo can resolve assets from downloaded Fuel worlds
    fuel_cache = os.path.expanduser('~/.gz/fuel')
    if fuel_cache not in os.environ.get('GZ_SIM_RESOURCE_PATH', ''):
        os.environ['GZ_SIM_RESOURCE_PATH'] = \
            os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + fuel_cache

    # ------------------------------------------------------------------ #
    # 1. Parse URDF (xacro → string)
    # ------------------------------------------------------------------ #
    urdf_path = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    robot_description = subprocess.check_output(['xacro', urdf_path]).decode()

    world_file  = os.path.join(pkg, 'worlds', 'maze2.sdf')

    # CHANGED: use slam_rviz.rviz so the map, scan, costmaps, and planned
    # path are all visible during autonomous operation.
    # Swap back to 'amr.rviz' if you prefer the original view.
    rviz_config = os.path.join(pkg, 'rviz', 'slam_rviz.rviz')

    # ------------------------------------------------------------------ #
    # 2. Gazebo Harmonic
    # ------------------------------------------------------------------ #
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file} -r -v 4',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 3. Robot State Publisher
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------ #
    # 4. Spawn robot into Gazebo
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        output='screen',
        arguments=[
            '-name',  'amr',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ],
    )

    # ------------------------------------------------------------------ #
    # 5. Controllers
    # ------------------------------------------------------------------ #
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Wait for robot spawn, then load joint_state_broadcaster
    activate_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    # Wait for joint_state_broadcaster, then load diff_drive_controller
    activate_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[diff_drive_controller_spawner],
                )
            ],
        )
    )

    # ------------------------------------------------------------------ #
    # 6. ROS ↔ Gazebo topic bridges
    # /scan, /imu, /clock are required by SLAM Toolbox and Nav2.
    # ------------------------------------------------------------------ #
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # ------------------------------------------------------------------ #
    # 7. EKF — fuses wheel odometry + IMU → /odometry/filtered
    #    and publishes the corrected odom → base_link TF.
    #    Nav2 reads /odometry/filtered (set in nav2_params.yaml).
    # ------------------------------------------------------------------ #
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ekf.launch.py')
        )
    )

    # ------------------------------------------------------------------ #
    # 8. RViz  (delayed 5 s to let everything start up first)
    # ------------------------------------------------------------------ #
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        activate_jsb,
        activate_ddc,
        bridge,
        ekf_launch,
        rviz,
    ])