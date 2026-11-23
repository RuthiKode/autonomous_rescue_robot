"""Frontier extraction and scoring on an occupancy grid. Pure numpy/scipy, no ROS.

Grid convention: a 2D int array, row 0 = the lowest y (the OccupancyGrid message layout);
value -1 unknown, 0..free_max free, >= occupied_min occupied. Cell (row, col) has its centre at
(origin_x + (col + 0.5) * res, origin_y + (row + 0.5) * res).
"""
import math

import numpy as np
from scipy import ndimage

EIGHT = np.ones((3, 3), dtype=bool)


class Cluster:
    """A connected group of frontier cells and the goal chosen for it."""

    def __init__(self, cells, goal_cell, size_m):
        self.cells = cells              # (n, 2) array of (row, col)
        self.goal_cell = goal_cell      # (row, col) of a free, clear cell in the cluster
        self.size_m = size_m
        self.gain_m2 = 0.0
        self.hazard_m2 = 0.0


def frontier_mask(grid, free_max=25):
    """Free cells that touch an unknown cell (8-neighbourhood)."""
    unknown = grid < 0
    free = (grid >= 0) & (grid <= free_max)
    return free & ndimage.binary_dilation(unknown, structure=EIGHT)


def clearance_m(grid, res, occupied_min=65):
    """Distance in metres from every cell to the nearest occupied cell."""
    return ndimage.distance_transform_edt(grid < occupied_min) * res


def extract_clusters(grid, res, min_size_m=0.5, goal_clearance_m=0.35, free_max=25,
                     occupied_min=65, max_clusters=60):
    """Connected frontier clusters of at least min_size_m, each with a goal cell.

    The goal is the cluster cell nearest the cluster centroid among those at least
    goal_clearance_m from an occupied cell; a cluster with no such cell is dropped (Nav2 would
    refuse a goal that close to a wall anyway). Size is the number of cells times the
    resolution, which is the frontier length for a straight frontier.
    """
    mask = frontier_mask(grid, free_max)
    labels, n = ndimage.label(mask, structure=EIGHT)
    if n == 0:
        return []
    clear = clearance_m(grid, res, occupied_min)
    clusters = []
    for objs, lab in zip(ndimage.find_objects(labels), range(1, n + 1)):
        rows, cols = np.nonzero(labels[objs] == lab)
        rows, cols = rows + objs[0].start, cols + objs[1].start
        size = len(rows) * res
        if size < min_size_m:
            continue
        ok = clear[rows, cols] >= goal_clearance_m
        if not ok.any():
            continue
        rows, cols = rows[ok], cols[ok]
        cr, cc = rows.mean(), cols.mean()
        k = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
        clusters.append(Cluster(np.stack([rows, cols], axis=1), (int(rows[k]), int(cols[k])), size))
    clusters.sort(key=lambda c: -c.size_m)
    return clusters[:max_clusters]


def unknown_area_m2(grid, cell, res, radius_m):
    """Unknown area within radius_m of a cell (a disc, in square metres)."""
    r = int(math.ceil(radius_m / res))
    r0, r1 = max(cell[0] - r, 0), min(cell[0] + r + 1, grid.shape[0])
    c0, c1 = max(cell[1] - r, 0), min(cell[1] + r + 1, grid.shape[1])
    window = grid[r0:r1, c0:c1] < 0
    rr, cc = np.ogrid[r0:r1, c0:c1]
    disc = (rr - cell[0]) ** 2 + (cc - cell[1]) ** 2 <= (radius_m / res) ** 2
    return float(np.count_nonzero(window & disc)) * res * res


def hazard_area_m2(hazard, cell, res, radius_m):
    """Hazard cells (value 100) within radius_m of a cell, in square metres."""
    if hazard is None:
        return 0.0
    r = int(math.ceil(radius_m / res))
    r0, r1 = max(cell[0] - r, 0), min(cell[0] + r + 1, hazard.shape[0])
    c0, c1 = max(cell[1] - r, 0), min(cell[1] + r + 1, hazard.shape[1])
    rr, cc = np.ogrid[r0:r1, c0:c1]
    disc = (rr - cell[0]) ** 2 + (cc - cell[1]) ** 2 <= (radius_m / res) ** 2
    return float(np.count_nonzero((hazard[r0:r1, c0:c1] == 100) & disc)) * res * res


def cell_to_xy(cell, res, origin):
    return origin[0] + (cell[1] + 0.5) * res, origin[1] + (cell[0] + 0.5) * res


def xy_to_cell(x, y, res, origin):
    return int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))


def score(gain_m2, path_length_m, hazard_m2, min_path_m=1.0, hazard_weight=2.0):
    """Information gain per metre of path, minus a penalty for hazard area near the goal.

    gain_m2 / max(path, min_path) has the unit of a length; hazard_weight turns square metres
    of hazard near the goal into the same unit. min_path_m stops a goal at the robot's feet
    from scoring infinity.
    """
    return gain_m2 / max(path_length_m, min_path_m) - hazard_weight * hazard_m2


def blacklisted(x, y, blacklist, radius_m):
    return any(math.hypot(x - bx, y - by) <= radius_m for bx, by in blacklist)


def goal_heading(grid, cell, cluster_cells, res):
    """Yaw pointing from the goal towards the nearest unknown cell, so the robot ends facing it."""
    r = int(round(1.5 / res))
    r0, r1 = max(cell[0] - r, 0), min(cell[0] + r + 1, grid.shape[0])
    c0, c1 = max(cell[1] - r, 0), min(cell[1] + r + 1, grid.shape[1])
    rows, cols = np.nonzero(grid[r0:r1, c0:c1] < 0)
    if rows.size == 0:
        return 0.0
    rows, cols = rows + r0, cols + c0
    k = int(np.argmin((rows - cell[0]) ** 2 + (cols - cell[1]) ** 2))
    return math.atan2(rows[k] - cell[0], cols[k] - cell[1])
