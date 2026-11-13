"""The bridge YAML must cover the Phase 1 topic contract and must not bridge odom TF."""
import os

import yaml

with open(os.path.join(os.path.dirname(__file__), '..', 'config', 'bridge.yaml')) as _f:
    BRIDGE = {e['ros_topic_name']: e for e in yaml.safe_load(_f)}

CONTRACT = {
    '/clock': ('rosgraph_msgs/msg/Clock', 'GZ_TO_ROS'),
    '/scan': ('sensor_msgs/msg/LaserScan', 'GZ_TO_ROS'),
    # /imu and /odom_raw come from covariance_stamper, fed by these two:
    '/gz/imu': ('sensor_msgs/msg/Imu', 'GZ_TO_ROS'),
    '/gz/odom_raw': ('nav_msgs/msg/Odometry', 'GZ_TO_ROS'),
    '/ground_truth/odom': ('nav_msgs/msg/Odometry', 'GZ_TO_ROS'),
    '/camera/image_raw': ('sensor_msgs/msg/Image', 'GZ_TO_ROS'),
    '/camera/depth': ('sensor_msgs/msg/Image', 'GZ_TO_ROS'),
    '/camera/camera_info': ('sensor_msgs/msg/CameraInfo', 'GZ_TO_ROS'),
    '/cmd_vel': ('geometry_msgs/msg/Twist', 'ROS_TO_GZ'),
}


def test_contract_topics_are_bridged():
    for topic, (ros_type, direction) in CONTRACT.items():
        assert topic in BRIDGE, topic
        assert BRIDGE[topic]['ros_type_name'] == ros_type
        assert BRIDGE[topic]['direction'] == direction


def test_tf_is_not_bridged():
    assert '/tf' not in BRIDGE
    assert all('tf2_msgs' not in e['ros_type_name'] for e in BRIDGE.values())
