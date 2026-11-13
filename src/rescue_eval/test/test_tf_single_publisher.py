"""Integration test: with the full mapping stack up, every transform has one publisher.

Starts Gazebo (headless), the bridge, the EKF and slam_toolbox, listens to /tf
and /tf_static, and fails if any transform is published by more than one node
or if the map -> odom -> base_link -> sensor chain is incomplete.

Needs Gazebo and a GPU or software renderer. Set RESCUE_SKIP_SIM_TESTS=1 to skip.
"""
import os
import tempfile

import pytest
import rclpy

from rescue_eval.sim_runner import launch_sim, spin_until
from rescue_eval.tf_audit import TfAudit

LISTEN_WALL_S = 8.0
EXPECTED_CHAIN = [('map', 'odom'), ('odom', 'base_link'), ('base_link', 'laser_link'),
                  ('base_link', 'camera_link'), ('base_link', 'imu_link')]


@pytest.mark.skipif(os.environ.get('RESCUE_SKIP_SIM_TESTS') == '1',
                    reason='RESCUE_SKIP_SIM_TESTS=1')
def test_every_transform_has_one_publisher():
    # A private ROS domain and Gazebo partition keep other running sims out of the test.
    env = dict(os.environ, ROS_DOMAIN_ID='87', GZ_PARTITION='rescue_tf_test')
    os.environ.update(ROS_DOMAIN_ID='87')
    log_path = os.path.join(tempfile.gettempdir(), 'test_tf_single_publisher.launch.log')
    sim = launch_sim('rooms', 0, log_path, launch_file='mapping.launch.py', env=env)
    rclpy.init()
    node = rclpy.create_node('tf_single_publisher_test')
    audit = TfAudit(node)
    try:
        def chain_complete():
            seen = {(p, c) for p, c, _ in audit.transforms()}
            return all(link in seen for link in EXPECTED_CHAIN)

        result = spin_until(node, chain_complete, 150.0, sim.alive)
        assert result == 'ok', (f'TF chain never completed ({result}).\n{audit.report()}\n'
                                f'launch log: {log_path}')
        audit.observations.clear()   # judge a clean window with every publisher running
        spin_until(node, lambda: False, LISTEN_WALL_S, sim.alive)

        print(audit.report())
        frames = audit.frames()
        assert frames['odom']['parent'] == 'map'
        assert frames['base_link']['parent'] == 'odom'
        assert frames['laser_link']['parent'] == 'base_link'
        seen = {(p, c) for p, c, _ in audit.transforms()}
        assert ('map', 'odom') in seen and ('odom', 'base_link') in seen
        assert audit.violations() == []
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        sim.stop()
