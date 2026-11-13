"""The Xacro model must expand and contain the frames and sensors the plan requires."""
import os
import xml.etree.ElementTree as ET

import xacro

XACRO = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'rescue_robot.urdf.xacro')


def _robot():
    return ET.fromstring(xacro.process_file(XACRO).toxml())


def test_frames_match_tf_contract():
    robot = _robot()
    links = {link.get('name') for link in robot.findall('link')}
    assert {'base_link', 'laser_link', 'camera_link', 'imu_link'} <= links
    parents = {j.find('child').get('link'): j.find('parent').get('link')
               for j in robot.findall('joint')}
    for child in ('laser_link', 'camera_link', 'imu_link'):
        assert parents[child] == 'base_link'


def test_sensors_have_rates_and_noise():
    robot = _robot()
    sensors = {s.get('name'): s for s in robot.iter('sensor')}
    expected = {'lidar': 10.0, 'rgb_camera': 15.0, 'depth_camera': 15.0, 'imu': 50.0}
    assert set(expected) <= set(sensors)
    for name, rate in expected.items():
        assert float(sensors[name].find('update_rate').text) == rate
        stddevs = [float(e.text) for e in sensors[name].iter('stddev')]
        assert stddevs and all(s > 0.0 for s in stddevs), name


def test_diff_drive_tf_is_not_on_tf_topic():
    robot = _robot()
    plugins = [p for p in robot.iter('plugin') if 'diff-drive' in p.get('filename')]
    assert len(plugins) == 1
    tf_topic = plugins[0].find('tf_topic').text
    assert tf_topic.strip('/') != 'tf'


def test_wheels_have_low_friction():
    robot = _robot()
    mus = {g.get('reference'): float(g.find('mu1').text)
           for g in robot.findall('gazebo') if g.find('mu1') is not None}
    assert 0.0 < mus['left_wheel'] < 1.0
    assert mus['left_wheel'] == mus['right_wheel']
