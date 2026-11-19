"""The Nav2 parameters must match the robot and the topic contract."""
import ast
import os
import re

import yaml

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'nav2_params.yaml')) as _f:
    P = yaml.safe_load(_f)
with open(os.path.join(HERE, '..', '..', 'rescue_description', 'urdf', 'robot_params.xacro')) as _f:
    XACRO = _f.read()


def _x(name):
    return float(re.search(rf'name="{name}" value="([^"]+)"', XACRO).group(1))


def _footprint(costmap):
    return ast.literal_eval(P[costmap][costmap]['ros__parameters']['footprint'])


def test_no_map_server_and_no_amcl():
    assert 'amcl' not in P and 'map_server' not in P and 'loopback_simulator' not in P


def test_footprint_matches_the_xacro_chassis_in_both_costmaps():
    half_l, half_w, off = _x('chassis_length') / 2, _x('chassis_width') / 2, _x('chassis_offset_x')
    expected = sorted([[off + half_l, half_w], [off + half_l, -half_w],
                       [off - half_l, -half_w], [off - half_l, half_w]])
    for costmap in ('local_costmap', 'global_costmap'):
        assert sorted(_footprint(costmap)) == [[round(v, 3) for v in p] for p in expected]
        assert 'robot_radius' not in P[costmap][costmap]['ros__parameters']


def test_inflation_is_circumscribed_radius_plus_margin():
    xs = [p[0] for p in _footprint('global_costmap')]
    ys = [p[1] for p in _footprint('global_costmap')]
    circumscribed = max((x * x + y * y) ** 0.5 for x, y in zip(xs, ys))
    for costmap, layer in (('local_costmap', 'inflation_layer'), ('global_costmap', 'inflation_layer')):
        r = P[costmap][costmap]['ros__parameters'][layer]['inflation_radius']
        assert 0.10 <= r - circumscribed <= 0.20, (costmap, r, circumscribed)


def test_controller_limits_do_not_exceed_the_robot():
    ctrl = P['controller_server']['ros__parameters']['FollowPath']
    assert ctrl['vx_max'] <= _x('max_linear_velocity')
    assert ctrl['wz_max'] <= _x('max_angular_velocity')
    assert ctrl['ax_max'] <= _x('max_linear_acceleration')
    assert ctrl['az_max'] <= _x('max_angular_acceleration')
    smoother = P['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] <= _x('max_linear_velocity')
    assert smoother['max_accel'][0] <= _x('max_linear_acceleration')
    assert smoother['max_accel'][2] <= _x('max_angular_acceleration')


def test_topics_and_frames_follow_the_contract():
    assert P['bt_navigator']['ros__parameters']['odom_topic'] == '/odometry/filtered'
    assert P['velocity_smoother']['ros__parameters']['odom_topic'] == 'odometry/filtered'
    assert P['collision_monitor']['ros__parameters']['base_frame_id'] == 'base_link'
    assert P['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert P['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == 'odom'


def test_the_controller_reads_speed_from_the_ekf_output():
    # Nav2's default is /odom, which does not exist here. With no odometry MPPI believed
    # the robot was always at rest and could only add one step of speed (1.5 m/s^2 x 0.05 s).
    ctrl = P['controller_server']['ros__parameters']
    assert ctrl['odom_topic'] == '/odometry/filtered'
    assert ctrl['FollowPath']['CostCritic']['consider_footprint'] is True


def test_the_depth_camera_is_an_obstacle_source_in_both_costmaps():
    # A lying survivor is below the LiDAR plane; the depth camera is what sees it.
    lidar_z = _x('lidar_z')
    layers = {
        'local_costmap': P['local_costmap']['local_costmap']['ros__parameters']['voxel_layer'],
        'global_costmap': P['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer'],
    }
    for name, layer in layers.items():
        assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer', name   # 3D, so LiDAR rays keep low marks
        assert 'depth' in layer['observation_sources'].split(), name
        depth = layer['depth']
        assert depth['topic'] == '/camera/depth/points' and depth['data_type'] == 'PointCloud2'
        # above the floor noise, and covering the heights the LiDAR plane misses
        assert depth['min_obstacle_height'] >= 0.03
        assert depth['max_obstacle_height'] > lidar_z
        assert depth['obstacle_min_range'] >= _x('camera_clip_near')
        assert depth['obstacle_max_range'] <= _x('camera_clip_far')


def test_recovery_behaviours_are_enabled():
    plugins = P['behavior_server']['ros__parameters']['behavior_plugins']
    assert {'spin', 'backup', 'wait'} <= set(plugins)
