"""The hazard layer on synthetic maps with known passage widths."""
import os
import re

import numpy as np
import pytest
import yaml

from rescue_mapping.hazard_layer import hazard_cells, hazard_grid

RES = 0.05
HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'hazard.yaml')) as _f:
    CFG = yaml.safe_load(_f)
PARAMS = dict(robot_width_m=CFG['robot_width_m'], margin_m=CFG['margin_m'],
              min_area_m2=CFG['min_area_m2'], occupied_threshold=CFG['occupied_threshold'])


def cells(m):
    return int(round(m / RES))


def room(width_m, height_m):
    """Free floor inside a one-cell-thick wall, in cells (row 0 = top)."""
    g = np.full((cells(height_m), cells(width_m)), 0, dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    return g


def two_rooms(gap_m, thickness_m=0.25, width_m=10.0, height_m=5.0):
    """Two rooms divided by a wall with one centred gap."""
    g = room(width_m, height_m)
    c0 = cells(width_m / 2 - thickness_m / 2)
    c1 = c0 + cells(thickness_m)
    g[:, c0:c1] = 100
    mid = g.shape[0] // 2
    half = cells(gap_m) // 2
    g[mid - half: mid + half, c0:c1] = 0
    return g, (c0, c1, mid, half)


def hazards(g, **overrides):
    return hazard_cells(g, RES, **{**PARAMS, **overrides})


def test_threshold_is_the_robot_width_plus_margin():
    assert CFG['robot_width_m'] + CFG['margin_m'] == pytest.approx(0.79)


def test_robot_width_is_the_circumscribed_diameter_from_the_xacro():
    with open(os.path.join(HERE, '..', '..', 'rescue_description', 'urdf', 'robot_params.xacro')) as f:
        x = f.read()

    def v(n):
        return float(re.search(rf'name="{n}" value="([^"]+)"', x).group(1))
    half_l, half_w, off = v('chassis_length') / 2, v('chassis_width') / 2, v('chassis_offset_x')
    radius = max(((off + sx * half_l) ** 2 + half_w ** 2) ** 0.5 for sx in (-1, 1))
    assert CFG['robot_width_m'] == pytest.approx(2 * radius, abs=0.01)


def test_a_narrow_gap_is_a_hazard_and_marks_the_gap_cells():
    g, (c0, c1, mid, half) = two_rooms(0.7)
    h = hazards(g)
    assert h[mid, (c0 + c1) // 2]                     # the middle of the gap
    assert h.sum() * RES ** 2 >= CFG['min_area_m2']
    assert not h[mid, cells(1.0)] and not h[mid, g.shape[1] - cells(1.0)]   # far from the gap
    assert not (h & (g == 100)).any()                 # never on obstacles


@pytest.mark.parametrize('gap', [0.9, 1.0, 1.5])
def test_wide_gaps_are_not_hazards(gap):
    g, _ = two_rooms(gap)
    assert not hazards(g).any()


def test_an_empty_room_has_no_hazards_even_in_the_corners():
    assert not hazards(room(6.0, 4.0)).any()


def test_a_wide_corridor_is_clear_and_a_narrow_one_is_not():
    for width, expected in ((2.0, False), (0.5, True)):
        g = np.full((cells(width) + 2, cells(8.0)), 0, dtype=np.int16)
        g[0, :] = g[-1, :] = 100
        # open both ends into big rooms
        big = np.full((cells(5.0), cells(4.0)), 0, dtype=np.int16)
        big[0, :] = big[-1, :] = 100
        big[:, 0] = 100
        left = big.copy()
        right = big[:, ::-1].copy()
        top = (left.shape[0] - g.shape[0]) // 2
        grid = np.full((left.shape[0], left.shape[1] * 2 + g.shape[1]), 100, dtype=np.int16)
        grid[:, :left.shape[1]] = left
        grid[:, -right.shape[1]:] = right
        grid[top: top + g.shape[0], left.shape[1]: left.shape[1] + g.shape[1]] = g
        # open the corridor ends
        grid[top + 1: top + g.shape[0] - 1, left.shape[1] - 1: left.shape[1] + 1] = 0
        grid[top + 1: top + g.shape[0] - 1, left.shape[1] + g.shape[1] - 1: left.shape[1] + g.shape[1] + 1] = 0
        assert bool(hazards(grid).any()) == expected, width


def test_a_gap_is_still_found_when_the_rooms_are_also_joined_elsewhere():
    g, (c0, c1, mid, half) = two_rooms(0.7, height_m=8.0)
    g[3:cells(1.0) + 3, c0:c1] = 0                    # a wide opening far from the gap
    h = hazards(g)
    assert h[mid, (c0 + c1) // 2]
    assert not h[3 + cells(0.5), (c0 + c1) // 2]


def test_unknown_cells_are_not_hazards_and_do_not_make_a_corridor_narrow():
    g = room(6.0, 4.0)
    g[:, 2 * g.shape[1] // 3:] = -1                   # a third of the room unexplored
    assert not hazards(g).any()
    out = hazard_grid(g, RES, **PARAMS)
    assert set(np.unique(out)) <= {-1, 0, 100}
    assert (out[g == -1] == -1).all() and (out[g == 100] == -1).all()


def test_the_published_grid_marks_the_gap_with_100():
    g, (c0, c1, mid, half) = two_rooms(0.7)
    out = hazard_grid(g, RES, **PARAMS)
    assert out[mid, (c0 + c1) // 2] == 100
    assert out[mid, cells(1.0)] == 0
