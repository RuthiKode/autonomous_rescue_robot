"""The SLAM parameters must match the plan and the robot's LiDAR."""
import os
import re

import yaml

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'slam_toolbox.yaml')) as _f:
    PARAMS = yaml.safe_load(_f)['slam_toolbox']['ros__parameters']
with open(os.path.join(HERE, '..', '..', 'rescue_description', 'urdf', 'robot_params.xacro')) as _f:
    XACRO = _f.read()


def _xacro_value(name):
    return float(re.search(rf'name="{name}" value="([^"]+)"', XACRO).group(1))


def test_resolution_and_loop_closure():
    assert PARAMS['resolution'] == 0.05
    assert PARAMS['do_loop_closing'] is True
    assert PARAMS['mode'] == 'mapping'


def test_laser_range_matches_the_lidar():
    assert PARAMS['max_laser_range'] == _xacro_value('lidar_range_max')
    assert PARAMS['min_laser_range'] == _xacro_value('lidar_range_min')


def test_frames_match_the_tf_contract():
    assert (PARAMS['map_frame'], PARAMS['odom_frame'], PARAMS['base_frame']) == (
        'map', 'odom', 'base_link')
    assert PARAMS['scan_topic'] == '/scan'
