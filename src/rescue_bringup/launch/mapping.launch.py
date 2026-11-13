"""Simulation plus SLAM: sim.launch.py with slam_toolbox on top.

  ros2 launch rescue_bringup mapping.launch.py world:=rooms headless:=false rviz:=true

TF chain: map -> odom (slam_toolbox) -> base_link (EKF) -> sensors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('rescue_bringup')
    slam = get_package_share_directory('rescue_slam')
    names = ['world', 'headless', 'seed', 'rviz', 'rviz_config']
    defaults = {'world': 'rooms', 'headless': 'false', 'seed': '0', 'rviz': 'false',
                'rviz_config': 'phase2.rviz'}
    sim_args = {name: LaunchConfiguration(name) for name in names}
    # The EKF must run, and the static map -> odom stand-in must not: SLAM owns that transform.
    sim_args.update({'ekf': 'true', 'static_map_tf': 'false'})
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=defaults[name]) for name in names] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
                launch_arguments=sim_args.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'slam.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items(),
            ),
        ])
