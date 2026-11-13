"""Start slam_toolbox (online async) with the project's parameters.

  ros2 launch rescue_slam slam.launch.py

Needs /scan and odom -> base_link (the EKF) to be running already. The node is
a lifecycle node; slam_toolbox's own launch file configures and activates it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('rescue_slam'),
                                  'config', 'slam_toolbox.yaml')
    upstream = os.path.join(get_package_share_directory('slam_toolbox'),
                            'launch', 'online_async_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
            }.items(),
        ),
    ])
