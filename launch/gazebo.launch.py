"""
gazebo.launch.py
----------------
Starts Gazebo Harmonic (gz sim) with the empty world.
The robot is spawned separately via spawn.launch.py,
or you can include this file inside a combined launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg = get_package_share_directory('amr_description')

    # Path to the empty world SDF
    world_file = os.path.join(pkg, 'worlds', 'empty.world')

    # gz_ros2_bridge launch — provided by ros_gz_sim package
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file} -r -v 4',
            # -r  = run immediately (don't pause)
            # -v 4 = verbose level 4 (useful for debugging)
        }.items(),
    )

    return LaunchDescription([
        gz_sim,
    ])
