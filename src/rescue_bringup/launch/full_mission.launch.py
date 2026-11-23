"""The whole system in one launch: simulation, SLAM, Nav2, perception, exploration and mission tree.

  ros2 launch rescue_bringup full_mission.launch.py world:=rooms detector:=yolo \
      model:=$HOME/autonomous_rescue_robot/results/detector/runs/all_x/weights/best.pt verify:=true

Order: simulation + EKF + slam_toolbox start together; `wakeup` drives forward a few seconds to seed
slam_toolbox's map and exits; when it has exited, Nav2, the perception nodes, the frontier selector
and the mission tree start. (Nav2 started earlier would fight the open-loop wake-up drive.)

Arguments
  world       rooms | collapsed | open_plan
  headless    true (default) runs Gazebo without a window
  seed        Gazebo random seed
  detector    yolo | oracle | none     (oracle is a ground-truth test double, never a result)
  model       weights file for detector:=yolo ('' = the pretrained COCO weights from perception.yaml)
  explorer    own (frontier selector + mission tree) | lite (explore_lite) | random (random goals)
  verify      true: the robot drives to pins for a close look; the localizer verifies or rejects them
  foxglove    true: also start foxglove_bridge (ws://localhost:8765) for the layout in foxglove/
  rviz        true: also start RViz2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include(package, launch_file, condition=None, **arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    arg = LaunchConfiguration

    def when(expression):
        return IfCondition(PythonExpression(expression))

    wakeup = Node(package='rescue_mission', executable='wakeup', output='screen',
                  parameters=[{'use_sim_time': True}])
    started_after_wakeup = [
        include('rescue_navigation', 'navigation.launch.py'),
        include('rescue_perception', 'perception.launch.py', condition=when(["'", arg('detector'), "' != 'none'"]),
                detector=arg('detector'), world=arg('world'), model=arg('model'),
                verification=arg('verify')),
        include('rescue_mission', 'mission.launch.py',
                condition=when(["'", arg('explorer'), "' in ('own', 'random')"]),
                verify=arg('verify'),
                selector_mode=PythonExpression(["'random' if '", arg('explorer'),
                                                "' == 'random' else 'score'"])),
        include('rescue_exploration', 'explore_baseline.launch.py',
                condition=when(["'", arg('explorer'), "' == 'lite'"])),
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(arg('foxglove'))),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rooms'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('detector', default_value='yolo'),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('explorer', default_value='own'),
        DeclareLaunchArgument('verify', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        include('rescue_bringup', 'mapping.launch.py', world=arg('world'), headless=arg('headless'),
                seed=arg('seed'), rviz=arg('rviz')),
        wakeup,
        RegisterEventHandler(OnProcessExit(target_action=wakeup, on_exit=started_after_wakeup)),
    ])
