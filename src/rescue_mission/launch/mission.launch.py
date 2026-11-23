"""Own exploration: frontier selector plus the mission behaviour tree.

  ros2 launch rescue_mission mission.launch.py                # explore, recover
  ros2 launch rescue_mission mission.launch.py verify:=true   # also drive to pins for a close look

Start it on top of the simulation, SLAM and Nav2 (it replaces explore_baseline.launch.py). With
verify:=true the perception launch must also have verification:=true, so the localizer can turn the
close-range views into verified / rejected pins.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mission_cfg = os.path.join(get_package_share_directory('rescue_mission'), 'config',
                               'mission.yaml')
    selector_cfg = os.path.join(get_package_share_directory('rescue_exploration'), 'config',
                                'frontier_selector.yaml')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    verify = {'verify_enabled': ParameterValue(LaunchConfiguration('verify'), value_type=bool)}
    selector_mode = {'mode': LaunchConfiguration('selector_mode')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('verify', default_value='false'),
        DeclareLaunchArgument('selector_mode', default_value='score',
                              description='score: information gain per path cost; random: random '
                                          'reachable goals (the lower-bound baseline)'),
        Node(package='rescue_exploration', executable='frontier_selector',
             name='frontier_selector', output='screen',
             parameters=[selector_cfg, selector_mode, use_sim_time]),
        Node(package='rescue_mission', executable='mission_tree', name='mission_tree',
             output='screen', parameters=[mission_cfg, verify, use_sim_time]),
        Node(package='rescue_exploration', executable='map_stats', name='map_stats',
             output='screen', parameters=[use_sim_time]),
    ])
