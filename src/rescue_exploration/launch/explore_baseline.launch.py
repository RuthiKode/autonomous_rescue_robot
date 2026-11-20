"""explore_lite with the project's parameters.

  ros2 launch rescue_exploration explore_baseline.launch.py

Needs Nav2's navigate_to_pose action and slam_toolbox's /map to be up already.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rescue_exploration'),
                          'config', 'explore_lite.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=config),
        Node(
            package='explore_lite', executable='explore', name='explore_node', output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
