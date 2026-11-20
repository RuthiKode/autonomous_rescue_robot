"""The oracle's survivor shapes and projection, without ROS."""
import math

import pytest

from rescue_perception.geometry import inverse_transform_point
from rescue_perception.oracle_detector import survivor_points

OPTICAL = (-0.5, 0.5, -0.5, 0.5)         # camera_optical in base_link: z forward, x right, y down


def project(point, fx=500.0, cx=320.0, cy=240.0):
    x, y, z = point
    return fx * x / z + cx, fx * y / z + cy, z


def test_standing_survivor_ahead_projects_inside_the_image_and_is_taller_than_wide():
    # camera at the origin looking along +x at height 0.27; survivor 3 m ahead
    pts = survivor_points('standing', 3.0, 0.0, 0.0)
    cam = [inverse_transform_point((0.0, 0.0, 0.27), OPTICAL, p) for p in pts]
    assert all(c[2] > 0 for c in cam)                       # in front of the camera
    px = [project(c) for c in cam]
    us, vs = [p[0] for p in px], [p[1] for p in px]
    assert 280 < sum(us) / len(us) < 360                    # roughly centred horizontally
    assert (max(vs) - min(vs)) > 1.5 * (max(us) - min(us))  # a standing person is tall


def test_lying_survivor_is_wider_than_tall_and_low_in_the_image():
    pts = survivor_points('lying', 3.0, 0.0, math.pi / 2)    # body across the view
    cam = [inverse_transform_point((0.0, 0.0, 0.27), OPTICAL, p) for p in pts]
    px = [project(c) for c in cam]
    us, vs = [p[0] for p in px], [p[1] for p in px]
    assert (max(us) - min(us)) > 2.0 * (max(vs) - min(vs))
    assert min(vs) > 240                                     # below the horizon


def test_a_survivor_behind_the_camera_has_points_with_negative_depth():
    pts = survivor_points('standing', -3.0, 0.0, 0.0)
    cam = [inverse_transform_point((0.0, 0.0, 0.27), OPTICAL, p) for p in pts]
    assert all(c[2] < 0 for c in cam)
