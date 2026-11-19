"""Simulation + SLAM + Nav2: the stack of plan stage 3.

  ros2 launch rescue_bringup navigation_sim.launch.py world:=rooms headless:=true

TF chain: map -> odom (slam_toolbox) -> base_link (EKF). Goals go to Nav2's
NavigateToPose action; its output reaches Gazebo through /cmd_vel.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('rescue_bringup')
    navigation = get_package_share_directory('rescue_navigation')
    names = ['world', 'headless', 'seed', 'rviz', 'rviz_config']
    defaults = {'world': 'rooms', 'headless': 'false', 'seed': '0', 'rviz': 'false',
                'rviz_config': 'phase2.rviz'}
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=defaults[n]) for n in names] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, 'launch',
                                                           'mapping.launch.py')),
                launch_arguments={n: LaunchConfiguration(n) for n in names}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation, 'launch',
                                                           'navigation.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),
        ])
