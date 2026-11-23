"""Hit-rate confirmation and the camera visibility test, without ROS."""
import numpy as np
import pytest

from rescue_perception.geometry import point_visible
from rescue_perception.tracker import CANDIDATE, CONFIRMED, Tracker

K = dict(fx=500.0, fy=500.0, cx=320.0, cy=240.0)


def feed(tracker, hits, views, track=None):
    """`views` frames in view; the point is detected in the first `hits` of them."""
    for i in range(views):
        if track is not None:
            tracker.count_view(track)
        if i < hits:
            track = tracker.update(3.0, 2.0, 0.5, 0.8, 100.0 + i, (0.0 if i % 2 else 1.0, 0.0))
    return track


def test_rule_off_by_default():
    t = Tracker(confirm_hits=5, confirm_views=2, view_separation=0.5)
    track = feed(t, hits=6, views=200)
    assert track.state == CONFIRMED


def test_low_hit_rate_never_confirms_and_demotes():
    t = Tracker(confirm_hits=5, confirm_views=2, view_separation=0.5, min_hit_rate=0.3,
                rate_after_views=20)
    track = feed(t, hits=6, views=10)             # too few views for the rate rule: confirmed
    assert track.state == CONFIRMED
    for _ in range(60):                           # keeps being seen, never detected again
        t.count_view(track)
    assert track.hit_rate < 0.3 and track.state == CANDIDATE
    assert t.confirmed() == []


def test_high_hit_rate_stays_confirmed():
    t = Tracker(confirm_hits=5, confirm_views=2, view_separation=0.5, min_hit_rate=0.3)
    track = feed(t, hits=40, views=50)
    assert track.state == CONFIRMED and track.hit_rate > 0.7


def test_demoted_track_recovers_when_detected_again():
    t = Tracker(confirm_hits=5, confirm_views=2, view_separation=0.5, min_hit_rate=0.3)
    track = feed(t, hits=6, views=10)
    for _ in range(40):
        t.count_view(track)
    assert track.state == CANDIDATE
    for i in range(40):
        t.count_view(track)
        t.update(3.0, 2.0, 0.5, 0.8, 300.0 + i, (float(i % 2), 0.0))
    assert track.state == CONFIRMED


def test_point_visible_range_and_image_bounds():
    depth = np.full((480, 640), 4.0, dtype=np.float32)
    assert point_visible((0.0, 0.0, 3.0), depth, **K, min_range=0.4, max_range=6.0)
    assert not point_visible((0.0, 0.0, 7.0), depth, **K, min_range=0.4, max_range=6.0)
    assert not point_visible((0.0, 0.0, 0.2), depth, **K, min_range=0.4, max_range=6.0)
    assert not point_visible((3.0, 0.0, 3.0), depth, **K, min_range=0.4, max_range=6.0)  # off to the side


def test_point_visible_occlusion():
    depth = np.full((480, 640), 5.0, dtype=np.float32)
    depth[:, 300:340] = 1.5                       # a wall in front of the centre
    assert not point_visible((0.0, 0.0, 4.0), depth, **K, min_range=0.4, max_range=6.0)
    assert point_visible((0.0, 0.0, 1.4), depth, **K, min_range=0.4, max_range=6.0)   # in front of it
    depth[:] = np.nan
    assert point_visible((0.0, 0.0, 4.0), depth, **K, min_range=0.4, max_range=6.0)


def test_confirm_confidence_gate_and_demotion():
    t = Tracker(confirm_hits=3, confirm_views=1, confirm_confidence=0.6)
    for i in range(5):
        track = t.update(3.0, 2.0, 0.5, 0.4, 100.0 + i, (0.0, 0.0))    # many hits, weak detections
    assert track.state == CANDIDATE
    for i in range(20):
        track = t.update(3.0, 2.0, 0.5, 0.95, 200.0 + i, (0.0, 0.0))
    assert track.state == CONFIRMED
    for i in range(200):
        t.update(3.0, 2.0, 0.5, 0.05, 300.0 + i, (0.0, 0.0))            # confidence collapses
    assert track.state == CANDIDATE


def test_merge_keeps_only_the_strongest_track():
    t = Tracker(radius=0.5, confirm_hits=3, confirm_views=1, merge_radius=1.5)
    for i in range(10):
        t.update(0.0, 0.0, 0.5, 0.8, 100.0 + i, (0.0, 0.0))             # strong track
    for i in range(5):
        t.update(1.2, 0.0, 0.5, 0.8, 100.0 + i, (0.0, 0.0))             # weaker, 1.2 m away
    for i in range(5):
        t.update(5.0, 0.0, 0.5, 0.8, 100.0 + i, (0.0, 0.0))             # separate survivor
    assert len(t.confirmed()) == 3
    gone = t.suppressed()
    assert len(gone) == 1
    assert next(x for x in t.tracks if x.id in gone).x == pytest.approx(1.2, abs=0.01)


def _close_track(t, hits, views, range_m=2.0):
    """A track with `views` close-range frames of which `hits` carry a detection."""
    track = t.update(3.0, 2.0, 0.5, 0.8, 100.0, (0.0, 0.0), range_m=6.0)
    for i in range(views):
        t.count_view(track, range_m)
        if i < hits:
            t.update(3.0, 2.0, 0.5, 0.8, 101.0 + i, (float(i % 2), 0.0), range_m=range_m)
    return track


def test_close_views_with_most_frames_detected_verify():
    from rescue_perception.tracker import VERIFIED
    t = Tracker(confirm_hits=5, confirm_views=2, view_separation=0.5, verification=True)
    track = _close_track(t, hits=14, views=16)
    assert track.state == VERIFIED
    assert track in t.confirmed()                  # verified tracks still count as pins


def test_close_views_with_almost_no_detections_reject():
    from rescue_perception.tracker import REJECTED
    t = Tracker(confirm_hits=3, confirm_views=1, verification=True)
    track = t.update(3.0, 2.0, 0.5, 0.8, 100.0, (0.0, 0.0), range_m=6.0)
    for i in range(4):                             # confirmed from far away
        t.update(3.0, 2.0, 0.5, 0.8, 101.0 + i, (0.0, 0.0), range_m=6.0)
    assert track.state == CONFIRMED
    for _ in range(20):                            # nothing there when seen from close
        t.count_view(track, 1.8)
    assert track.state == REJECTED and track not in t.confirmed()
    for i in range(30):                            # rejected is final
        t.update(3.0, 2.0, 0.5, 0.9, 200.0 + i, (float(i % 2), 0.0), range_m=6.0)
    assert track.state == REJECTED


def test_far_views_never_verify_or_reject_and_the_rule_can_be_off():
    t = Tracker(confirm_hits=3, confirm_views=1, verification=True)
    track = _close_track(t, hits=0, views=0)
    for _ in range(50):
        t.count_view(track, 5.0)                   # beyond close_range
    assert track.close_views == 0 and track.state == CANDIDATE
    off = Tracker(confirm_hits=3, confirm_views=1)         # verification=False
    track2 = _close_track(off, hits=0, views=30)
    assert track2.close_views == 30 and track2.state == CANDIDATE
