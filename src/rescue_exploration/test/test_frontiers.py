"""Frontier extraction and scoring on small synthetic grids (no ROS)."""
import math

import numpy as np

from rescue_exploration import frontiers as fr

RES = 0.05


def room(h=100, w=100):
    """Known free square in the lower left, unknown everywhere else, walls on two sides."""
    g = np.full((h, w), -1, dtype=np.int16)
    g[10:60, 10:60] = 0
    g[10:60, 10] = 100               # left wall
    g[10, 10:60] = 100               # bottom wall
    return g


def test_frontier_cells_are_free_cells_next_to_unknown():
    g = room()
    mask = fr.frontier_mask(g)
    assert mask[59, 30] and mask[30, 59]          # open top and right edges
    assert not mask[30, 30]                       # interior
    assert not mask[10, 30] and not mask[30, 10]  # walls are not free


def test_clusters_have_a_clear_goal_and_respect_minimum_size():
    g = room()
    clusters = fr.extract_clusters(g, RES, min_size_m=0.5, goal_clearance_m=0.2)
    assert len(clusters) == 1                     # the top and right edges join at the corner
    c = clusters[0]
    assert c.size_m > 2.0
    assert g[c.goal_cell] == 0
    assert fr.clearance_m(g, RES)[c.goal_cell] >= 0.2
    assert fr.extract_clusters(g, RES, min_size_m=50.0) == []


def test_small_gap_is_dropped_but_a_long_frontier_stays():
    g = np.full((60, 60), 0, dtype=np.int16)
    g[:, 40:] = -1                                # long frontier at col 39
    g[20:23, 10:13] = -1                          # a 3x3 hole: tiny frontier ring
    sizes = sorted(c.size_m for c in fr.extract_clusters(g, RES, min_size_m=1.0, goal_clearance_m=0.1))
    assert len(sizes) == 1 and sizes[0] >= 1.0


def test_unknown_area_counts_a_disc():
    g = np.full((100, 100), -1, dtype=np.int16)
    area = fr.unknown_area_m2(g, (50, 50), RES, 1.0)
    assert abs(area - math.pi * 1.0 ** 2) < 0.15
    g[:, :] = 0
    assert fr.unknown_area_m2(g, (50, 50), RES, 1.0) == 0.0


def test_hazard_area_and_score_penalty():
    haz = np.zeros((100, 100), dtype=np.int16)
    haz[45:55, 45:55] = 100
    area = fr.hazard_area_m2(haz, (50, 50), RES, 1.0)
    assert abs(area - 100 * RES * RES) < 1e-9
    assert fr.hazard_area_m2(None, (50, 50), RES, 1.0) == 0.0
    clean = fr.score(8.0, 4.0, 0.0)
    assert clean == 2.0
    assert fr.score(8.0, 4.0, area) < clean


def test_score_prefers_gain_per_metre_and_does_not_blow_up_at_zero_path():
    assert fr.score(10.0, 2.0, 0.0) > fr.score(10.0, 8.0, 0.0)
    assert fr.score(10.0, 0.0, 0.0) == 10.0       # min_path_m = 1


def test_blacklist_radius_and_cell_conversion_round_trip():
    assert fr.blacklisted(1.0, 1.0, [(1.5, 1.0)], 1.0)
    assert not fr.blacklisted(1.0, 1.0, [(3.0, 1.0)], 1.0)
    origin = (-2.0, -3.0)
    cell = fr.xy_to_cell(0.123, 0.456, RES, origin)
    x, y = fr.cell_to_xy(cell, RES, origin)
    assert abs(x - 0.123) <= RES and abs(y - 0.456) <= RES


def test_goal_heading_points_at_the_unknown():
    g = np.zeros((60, 60), dtype=np.int16)
    g[:, 40:] = -1                                # unknown to the east (+col)
    yaw = fr.goal_heading(g, (30, 38), None, RES)
    assert abs(yaw) < 0.3
    g2 = np.zeros((60, 60), dtype=np.int16)
    g2[45:, :] = -1                               # unknown to the north (+row)
    assert abs(fr.goal_heading(g2, (43, 30), None, RES) - math.pi / 2) < 0.3


def test_map_stats_free_area():
    from rescue_exploration.map_stats_node import free_area_m2
    data = [-1, 0, 0, 100, 25, 26, 0, -1]
    assert abs(free_area_m2(data, 0.5) - 4 * 0.25) < 1e-9       # four cells with 0 <= value <= 25
