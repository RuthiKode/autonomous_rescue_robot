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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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
        DeclareLaunchArgument('depth_obstacles', default_value='true',
                              description='true converts the depth image to /camera/depth/points '
                                          'for the costmaps (low obstacles under the LiDAR plane)'),
        # depth image -> half-size depth image -> point cloud, for the costmaps' second
        # observation source. A full 640 x 480 cloud is 307,000 points per frame at 15 Hz and
        # was too heavy to deliver; half size is 76,800. Nearest-neighbour resizing (interpolation
        # 0) keeps depth values from being averaged across object edges. Both nodes share one
        # container with intra-process transport, so the images are not copied.
        ComposableNodeContainer(
            name='depth_points_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(LaunchConfiguration('depth_obstacles')),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::ResizeNode',
                    name='depth_resize',
                    remappings=[('image/image_raw', '/camera/depth'),
                                ('image/camera_info', '/camera/camera_info'),
                                ('resize/image_raw', '/camera/depth_small/image_raw'),
                                ('resize/camera_info', '/camera/depth_small/camera_info')],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                 'use_scale': True, 'scale_height': 0.5, 'scale_width': 0.5,
                                 'interpolation': 0}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='depth_points',
                    remappings=[('image_rect', '/camera/depth_small/image_raw'),
                                ('camera_info', '/camera/depth_small/camera_info'),
                                ('points', '/camera/depth/points')],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
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
