"""Start Gazebo with one generated world, the robot, the bridge, the EKF and ground truth.

  ros2 launch rescue_bringup sim.launch.py world:=rooms headless:=false

Drive the robot from a second terminal:

  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def _setup(context):
    world = LaunchConfiguration('world').perform(context)
    seed = LaunchConfiguration('seed').perform(context)

    sim_share = get_package_share_directory('rescue_simulation')
    desc_share = get_package_share_directory('rescue_description')
    bringup_share = get_package_share_directory('rescue_bringup')

    with open(os.path.join(bringup_share, 'config', 'sim.yaml')) as f:
        cfg = yaml.safe_load(f)

    world_file = os.path.join(sim_share, 'worlds', world + '.sdf')
    truth_file = os.path.join(sim_share, 'worlds', world + '_survivors.yaml')
    if not os.path.isfile(world_file):
        available = sorted(f[:-4] for f in os.listdir(os.path.join(sim_share, 'worlds'))
                           if f.endswith('.sdf'))
        raise RuntimeError(f"Unknown world '{world}'. Available: {', '.join(available)}")

    with open(truth_file) as f:
        truth = yaml.safe_load(f)
    spawn = truth['robot_spawn']

    gz_cmd = ['gz', 'sim', '-r', '--seed', seed]
    if _flag(context, 'headless'):
        gz_cmd.append('-s')
    gz_cmd.append(world_file)
    gz_env = {k: str(v) for k, v in cfg['gz_environment'].items() if str(v) != ''}

    sim_time = {'use_sim_time': True}
    xacro_file = os.path.join(desc_share, 'urdf', 'rescue_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    actions = [
        ExecuteProcess(cmd=gz_cmd, additional_env=gz_env, output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, sim_time],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-world', truth['world'],
                '-topic', 'robot_description',
                '-name', cfg['robot']['name'],
                '-x', str(spawn['x']),
                '-y', str(spawn['y']),
                '-z', str(spawn['z'] + cfg['robot']['spawn_z_offset']),
                '-Y', str(spawn['yaw']),
            ],
            parameters=[sim_time],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[
                {'config_file': os.path.join(sim_share, 'config', 'bridge.yaml')},
                sim_time,
            ],
        ),
        Node(
            package='rescue_simulation',
            executable='covariance_stamper',
            output='screen',
            parameters=[
                {'config_file': os.path.join(sim_share, 'config', 'sensor_covariances.yaml')},
                sim_time,
            ],
        ),
        Node(
            package='rescue_simulation',
            executable='ground_truth_publisher',
            output='screen',
            parameters=[{
                'truth_file': truth_file,
                'publish_rate_hz': float(cfg['ground_truth']['publish_rate_hz']),
                'topic': cfg['ground_truth']['topic'],
                'marker_topic': cfg['ground_truth']['marker_topic'],
            }, sim_time],
        ),
    ]

    if _flag(context, 'ekf'):
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml'), sim_time],
        ))

    if _flag(context, 'static_map_tf'):
        # Stand-in for SLAM so map-frame data can be viewed in Phase 1. Never
        # combine with slam_toolbox: both would publish map -> odom.
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[sim_time],
        ))

    if _flag(context, 'rviz'):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(bringup_share, 'rviz',
                                          LaunchConfiguration('rviz_config').perform(context))],
            parameters=[sim_time],
            additional_env=gz_env,
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='rooms',
                              description='World name: rooms, collapsed or open_plan'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true runs the Gazebo server only (no GUI)'),
        DeclareLaunchArgument('seed', default_value='0',
                              description='Gazebo random seed (sensor noise)'),
        DeclareLaunchArgument('ekf', default_value='true',
                              description='true starts the EKF that publishes odom -> base_link'),
        DeclareLaunchArgument('static_map_tf', default_value='false',
                              description='true publishes a fixed identity map -> odom '
                                          '(only without SLAM)'),
        DeclareLaunchArgument('rviz', default_value='false', description='true starts RViz2'),
        DeclareLaunchArgument('rviz_config', default_value='phase1.rviz',
                              description='file name inside rescue_bringup/rviz'),
        OpaqueFunction(function=_setup),
    ])
