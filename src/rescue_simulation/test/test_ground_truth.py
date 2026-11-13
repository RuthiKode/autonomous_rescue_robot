"""Ground-truth YAML must convert to a well-formed SurvivorArray."""
import math

from rescue_msgs.msg import Survivor
from rescue_simulation.ground_truth_publisher import survivors_from_truth


def test_conversion():
    truth = {'frame_id': 'map', 'survivors': [
        {'id': 1, 'posture': 'lying', 'occluded': True,
         'position': {'x': 1.5, 'y': -2.0, 'z': 0.11}, 'yaw': math.pi / 2},
        {'id': 2, 'posture': 'standing', 'occluded': False,
         'position': {'x': -3.0, 'y': 4.0, 'z': 0.85}, 'yaw': 0.0},
    ]}
    msg = survivors_from_truth(truth)
    assert msg.header.frame_id == 'map'
    assert [s.id for s in msg.survivors] == [1, 2]
    first = msg.survivors[0]
    assert (first.pose.position.x, first.pose.position.y) == (1.5, -2.0)
    assert math.isclose(first.pose.orientation.z, math.sin(math.pi / 4))
    assert math.isclose(first.pose.orientation.w, math.cos(math.pi / 4))
    assert first.state == Survivor.STATE_VERIFIED
    assert first.confidence == 1.0 and first.hit_count == 0
