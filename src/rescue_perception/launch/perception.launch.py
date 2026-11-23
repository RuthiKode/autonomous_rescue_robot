"""Detector and survivor localizer.

  ros2 launch rescue_perception perception.launch.py detector:=yolo
  ros2 launch rescue_perception perception.launch.py detector:=oracle world:=open_plan

detector:=yolo     pretrained YOLO in the YOLO virtual environment (needs PyTorch, ~/venvs/yolo)
detector:=oracle   ORACLE test double: detections from ground truth (needs world:=<name>).
                   Never report its output as a detection result.
detector:=none     only the localizer (something else publishes /detections)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    detector = LaunchConfiguration('detector').perform(context)
    world = LaunchConfiguration('world').perform(context)
    config = os.path.join(get_package_share_directory('rescue_perception'),
                          'config', 'perception.yaml')
    sim_time = {'use_sim_time': True}
    actions = []
    if detector == 'yolo':
        model = LaunchConfiguration('model').perform(context)
        actions.append(ExecuteProcess(
            cmd=[LaunchConfiguration('venv_python').perform(context), '-m',
                 'rescue_perception.yolo_detector', '--ros-args', '--params-file', config,
                 '-p', 'use_sim_time:=true'] + (['-p', f'model:={model}'] if model else []),
            name='yolo_detector', output='screen'))
    elif detector == 'oracle':
        truth = os.path.join(get_package_share_directory('rescue_simulation'), 'worlds',
                             world + '_survivors.yaml')
        actions.append(Node(package='rescue_perception', executable='oracle_detector',
                            output='screen',
                            parameters=[config, {'truth_file': truth}, sim_time]))
    elif detector != 'none':
        raise RuntimeError(f"detector must be yolo, oracle or none, not '{detector}'")
    overrides = {}
    for name in ('min_hit_rate', 'min_confidence'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)
    actions.append(Node(package='rescue_perception', executable='survivor_localizer',
                        output='screen', parameters=[config, overrides, sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detector', default_value='yolo'),
        DeclareLaunchArgument('world', default_value='rooms',
                              description='only used by detector:=oracle'),
        DeclareLaunchArgument('model', default_value='',
                              description='weights file for detector:=yolo (default: perception.yaml)'),
        DeclareLaunchArgument('min_hit_rate', default_value='',
                              description='localizer override (default: perception.yaml)'),
        DeclareLaunchArgument('min_confidence', default_value='',
                              description='localizer override (default: perception.yaml)'),
        DeclareLaunchArgument('venv_python',
                              default_value=os.path.expanduser('~/venvs/yolo/bin/python')),
        OpaqueFunction(function=_setup),
    ])
