"""Nav2's navigation launch on slam_toolbox's live map.

  ros2 launch rescue_navigation navigation.launch.py

Uses nav2_bringup's navigation_launch.py only: no map server and no AMCL, because
slam_toolbox already publishes /map and map -> odom. The simulation, the EKF and
slam_toolbox must already be running (rescue_bringup mapping.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('rescue_navigation'),
                                  'config', 'nav2_params.yaml')
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                        'navigation_launch.py')
    hazard_config = os.path.join(get_package_share_directory('rescue_mapping'),
                                 'config', 'hazard.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('hazard', default_value='true',
                              description='true also starts the hazard layer (/hazard_map)'),
        Node(
            package='rescue_mapping',
            executable='hazard_layer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('hazard')),
            parameters=[{'config_file': hazard_config},
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
            }.items(),
        ),
    ])
