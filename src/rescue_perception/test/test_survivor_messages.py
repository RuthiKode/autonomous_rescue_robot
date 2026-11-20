"""The /survivors and marker messages must be buildable from tracks (real ROS types)."""
import numpy as np
from builtin_interfaces.msg import Time

from rescue_perception.localizer_node import survivor_messages
from rescue_perception.tracker import CONFIRMED, Tracker


def make_tracks():
    t = Tracker(confirm_hits=3, confirm_views=2, view_separation=0.5)
    for i in range(4):                      # numpy scalars in, as the node produces them
        t.update(np.float32(3.0), np.float64(2.0), np.float32(0.5), np.float32(0.9),
                 100.0 + i, (np.float64(0.0 if i < 2 else 1.5), 0.0))
    t.update(8.0, 8.0, 0.2, 0.7, 101.0, (0.0, 0.0))
    return t.tracks


def test_messages_build_with_real_types_and_correct_content():
    tracks = make_tracks()
    msg, markers = survivor_messages(tracks, Time(sec=100, nanosec=500), 'map')
    assert msg.header.frame_id == 'map' and len(msg.survivors) == 2
    first = msg.survivors[0]
    assert first.state == CONFIRMED and first.hit_count == 4
    assert abs(first.pose.position.x - 3.0) < 1e-6 and abs(first.pose.position.y - 2.0) < 1e-6
    assert msg.survivors[1].state == 'candidate'
    assert len(markers.markers) == 4                         # a pin and a label per track
    assert any(m.text.startswith('#1 confirmed') for m in markers.markers)


def test_messages_serialize():
    from rclpy.serialization import serialize_message
    msg, markers = survivor_messages(make_tracks(), Time(), 'map')
    assert serialize_message(msg) and serialize_message(markers)


def test_no_tracks_gives_empty_messages():
    msg, markers = survivor_messages([], Time(), 'map')
    assert len(msg.survivors) == 0 and len(markers.markers) == 0
