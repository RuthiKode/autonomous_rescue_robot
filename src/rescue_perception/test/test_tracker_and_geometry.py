"""Tracker and depth geometry, without ROS."""
import math

import numpy as np
import pytest

from rescue_perception.geometry import (backproject, median_depth, quat_rotate,
                                        transform_point, yaw_from_quat)
from rescue_perception.tracker import CANDIDATE, CONFIRMED, Tracker


# ---------------- geometry ----------------
def test_median_depth_ignores_background_pixels_inside_the_box():
    depth = np.full((100, 100), 5.0, dtype=np.float32)           # far wall
    depth[30:70, 40:60] = 2.0                                      # the person
    # a box wider than the person: the central half is mostly person
    assert median_depth(depth, 50, 50, 40, 60, 0.5, 0.4, 6.0) == pytest.approx(2.0)


def test_median_depth_rejects_invalid_and_out_of_range_depth():
    depth = np.full((50, 50), np.nan, dtype=np.float32)
    assert median_depth(depth, 25, 25, 30, 30, 0.5, 0.4, 6.0) is None      # all NaN
    depth[:] = 0.2
    assert median_depth(depth, 25, 25, 30, 30, 0.5, 0.4, 6.0) is None      # too close
    depth[:] = 9.0
    assert median_depth(depth, 25, 25, 30, 30, 0.5, 0.4, 6.0) is None      # too far
    depth[:] = 3.0
    depth[20:30, 20:30] = np.inf
    assert median_depth(depth, 25, 25, 30, 30, 0.5, 0.4, 6.0) == pytest.approx(3.0)


def test_median_depth_clips_the_box_to_the_image():
    depth = np.full((40, 40), 2.5, dtype=np.float32)
    assert median_depth(depth, 38, 38, 30, 30, 1.0, 0.4, 6.0) == pytest.approx(2.5)
    assert median_depth(depth, -50, -50, 10, 10, 1.0, 0.4, 6.0) is None    # outside entirely


def test_backproject_uses_the_optical_convention():
    fx = fy = 500.0
    cx, cy = 320.0, 240.0
    assert backproject(cx, cy, 3.0, fx, fy, cx, cy) == (0.0, 0.0, 3.0)
    x, y, z = backproject(cx + 100, cy - 50, 2.0, fx, fy, cx, cy)
    assert (x, y, z) == (pytest.approx(0.4), pytest.approx(-0.2), 2.0)     # right is +x, up is -y


def test_transform_with_the_optical_to_base_rotation():
    # camera_optical in base_link: z forward -> +x, x right -> -y, y down -> -z
    s = math.sqrt(0.5)
    q = (-0.5, 0.5, -0.5, 0.5)                                             # rpy (-pi/2, 0, -pi/2)
    assert quat_rotate(q, (0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-9)  # forward
    assert quat_rotate(q, (1, 0, 0)) == pytest.approx((0, -1, 0), abs=1e-9)  # right
    assert quat_rotate(q, (0, 1, 0)) == pytest.approx((0, 0, -1), abs=1e-9)  # down
    p = transform_point((1.0, 2.0, 0.27), q, (0.0, 0.0, 3.0))
    assert p == pytest.approx((4.0, 2.0, 0.27), abs=1e-9)
    assert yaw_from_quat((0, 0, s, s)) == pytest.approx(math.pi / 2)


# ---------------- tracker ----------------
def make(**kw):
    return Tracker(radius=1.0, confirm_hits=5, confirm_views=2, view_separation=0.75,
                   expire_s=60.0, **kw)


def test_one_survivor_seen_many_times_is_one_track():
    t = make()
    for i in range(40):
        t.update(3.0 + 0.05 * math.sin(i), 2.0, 0.5, 0.8, float(i), (0.0, 0.0))
    assert len(t.tracks) == 1 and t.tracks[0].hits == 40


def test_two_survivors_further_apart_than_the_radius_are_two_tracks():
    t = make()
    t.update(0.0, 0.0, 0.5, 0.9, 0.0, (5.0, 0.0))
    t.update(1.5, 0.0, 0.5, 0.9, 1.0, (5.0, 0.0))
    assert len(t.tracks) == 2


def test_position_is_a_confidence_weighted_average():
    t = make()
    t.update(0.0, 0.0, 0.0, 0.9, 0.0, (0, 0))
    track = t.update(1.0, 0.0, 0.0, 0.3, 1.0, (0, 0))
    assert track.x == pytest.approx((0.0 * 0.9 + 1.0 * 0.3) / 1.2)
    assert track.confidence == pytest.approx(0.6)


def test_confirmation_needs_hits_and_distinct_viewpoints():
    t = make()
    for i in range(10):                       # many hits, one viewpoint: stays a candidate
        track = t.update(3.0, 3.0, 0.5, 0.8, float(i), (0.0, 0.0))
    assert track.state == CANDIDATE and track.hits == 10
    track = t.update(3.0, 3.0, 0.5, 0.8, 11.0, (0.5, 0.0))          # 0.5 m away: not distinct
    assert track.state == CANDIDATE
    track = t.update(3.0, 3.0, 0.5, 0.8, 12.0, (1.0, 0.0))          # 1.0 m away: second view
    assert track.state == CONFIRMED and len(track.views) == 2


def test_too_few_hits_do_not_confirm_even_from_two_viewpoints():
    t = make()
    t.update(3.0, 3.0, 0.5, 0.8, 0.0, (0.0, 0.0))
    track = t.update(3.0, 3.0, 0.5, 0.8, 1.0, (4.0, 0.0))
    assert track.state == CANDIDATE


def test_candidates_expire_but_confirmed_tracks_do_not():
    t = make()
    t.update(1.0, 1.0, 0.5, 0.8, 0.0, (0, 0))                       # a one-off false positive
    for i in range(6):
        t.update(6.0, 6.0, 0.5, 0.8, float(i), (0.0, 0.0) if i < 3 else (2.0, 0.0))
    assert len(t.tracks) == 2 and len(t.confirmed()) == 1
    dropped = t.expire(now=100.0)
    assert [d.id for d in dropped] == [1]
    assert len(t.tracks) == 1 and t.tracks[0].state == CONFIRMED


def test_a_late_hit_keeps_a_candidate_alive():
    t = make()
    t.update(1.0, 1.0, 0.5, 0.8, 0.0, (0, 0))
    t.update(1.0, 1.0, 0.5, 0.8, 50.0, (0, 0))
    assert t.expire(now=100.0) == []


def test_inverse_transform_undoes_the_transform():
    from rescue_perception.geometry import inverse_transform_point
    q = (-0.5, 0.5, -0.5, 0.5)
    t = (1.0, 2.0, 0.27)
    p = (3.0, -1.0, 0.5)
    assert inverse_transform_point(t, q, transform_point(t, q, p)) == pytest.approx(p, abs=1e-9)
