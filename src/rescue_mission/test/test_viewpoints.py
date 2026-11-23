import math

import numpy as np

from rescue_mission.viewpoints import choose_viewpoint, line_cells, line_of_sight

RES = 0.05


def open_room(n=200):
    g = np.zeros((n, n), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    return g


def test_line_cells_include_both_ends_and_are_connected():
    cells = line_cells(2, 3, 9, 20)
    assert cells[0] == (2, 3) and cells[-1] == (9, 20)
    assert all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in zip(cells[:-1], cells[1:]))


def test_wall_blocks_the_line_of_sight_but_unknown_does_not():
    g = open_room()
    assert line_of_sight(g, (50, 50), (50, 150))
    g[40:60, 100] = 100
    assert not line_of_sight(g, (50, 50), (50, 150))
    g[40:60, 100] = -1
    assert line_of_sight(g, (50, 50), (50, 150))


def test_viewpoint_is_close_clear_and_faces_the_target():
    g = open_room()
    origin = (-5.0, -5.0)
    target = (0.0, 0.0)
    vp = choose_viewpoint(g, RES, origin, target, robot_xy=(-4.0, 0.0))
    assert vp is not None
    x, y, yaw = vp
    d = math.hypot(x - target[0], y - target[1])
    assert 1.4 <= d <= 2.1
    assert abs(math.atan2(target[1] - y, target[0] - x) - yaw) < 1e-9
    assert x < 0                                      # the side nearest the robot


def test_viewpoint_skips_the_blocked_side_and_gives_up_when_boxed_in():
    g = open_room()
    origin = (-5.0, -5.0)
    g[:, 95:98] = 100                                 # a wall just left of the target (x = -0.25 .. -0.1)
    vp = choose_viewpoint(g, RES, origin, (0.0, 0.0), robot_xy=(-4.0, 0.0))
    assert vp is not None and vp[0] > 0               # forced to the far side
    boxed = np.full((200, 200), 100, dtype=np.int16)
    boxed[95:105, 95:105] = 0                         # only a 0.5 m pocket around the target
    assert choose_viewpoint(boxed, RES, origin, (0.0, 0.0), robot_xy=(0.0, 0.0)) is None
