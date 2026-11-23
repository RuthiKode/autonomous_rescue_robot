"""Verification viewpoints: where to stand to get a close, unobstructed look at a pin. No ROS.

Same grid convention as rescue_exploration.frontiers: int array, row 0 = lowest y, -1 unknown.
"""
import math

import numpy as np
from scipy import ndimage


def line_cells(r0, c0, r1, c1):
    """Cells on the straight line between two cells (Bresenham), both ends included."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            return cells
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def line_of_sight(grid, a, b, occupied_min=65):
    """True if no occupied cell lies on the line between cells a and b (unknown cells do not block)."""
    return all(grid[r, c] < occupied_min for r, c in line_cells(a[0], a[1], b[0], b[1]))


def choose_viewpoint(grid, res, origin, target_xy, robot_xy, radii=(1.5, 1.75, 2.0), n_angles=16,
                     clearance_m=0.35, free_max=25, occupied_min=65):
    """A free spot 1.5 to 2 m from target with a clear line of sight, nearest to the robot.

    Returns (x, y, yaw) with the heading towards the target, or None. Candidates are on circles
    of the given radii around the target, `n_angles` per circle.
    """
    h, w = grid.shape
    clear = ndimage.distance_transform_edt(grid < occupied_min) * res

    def cell(x, y):
        return int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))

    t_cell = cell(*target_xy)
    best, best_d = None, math.inf
    for radius in radii:
        for k in range(n_angles):
            a = 2 * math.pi * k / n_angles
            x, y = target_xy[0] + radius * math.cos(a), target_xy[1] + radius * math.sin(a)
            r, c = cell(x, y)
            if not (0 <= r < h and 0 <= c < w):
                continue
            if not (0 <= grid[r, c] <= free_max) or clear[r, c] < clearance_m:
                continue
            if not (0 <= t_cell[0] < h and 0 <= t_cell[1] < w):
                continue
            if not line_of_sight(grid, (r, c), t_cell, occupied_min):
                continue
            d = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if d < best_d:
                best, best_d = (x, y, math.atan2(target_xy[1] - y, target_xy[0] - x)), d
    return best
