"""Exploration coverage against the generator's ground truth.

Coverage = explored reachable free cells / all reachable free cells (docs/PLAN.md).

"Reachable free" cells are cells of the ground-truth grid (cut at the LiDAR height) that
the robot's centre can stand on (at least `robot_radius_m` from every obstacle) and that
are connected to the spawn. "Explored" is a cell of that set that the SLAM map marks free.
The SLAM map is resampled into the truth grid with the spawn pose as the map origin, as in
map_score.py.
"""
import numpy as np
from scipy import ndimage

from rescue_eval import map_score as ms


def reachable_truth_mask(truth, spawn, robot_radius_m=0.30):
    """Boolean array over the truth grid: free, robot-reachable cells connected to the spawn."""
    cells, res = truth['cells'], truth['resolution']
    free = cells == ms.FREE
    # distance to the nearest cell that is not free (obstacle or outside the building)
    standable = ndimage.distance_transform_edt(free) * res >= robot_radius_m
    h = cells.shape[0]
    col = int((spawn[0] - truth['origin'][0]) / res)
    row = h - 1 - int((spawn[1] - truth['origin'][1]) / res)
    labels, _ = ndimage.label(standable, structure=np.ones((3, 3)))
    if not (0 <= row < h and 0 <= col < cells.shape[1]) or labels[row, col] == 0:
        raise ValueError('the spawn is not on a standable free cell of the truth grid')
    return labels == labels[row, col]


def grid_to_map(msg):
    """nav_msgs/OccupancyGrid to the dict used by map_score (cells row 0 = top)."""
    data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    cells = np.full(data.shape, ms.UNKNOWN, dtype=np.int8)
    cells[data == 0] = ms.FREE
    cells[data >= 65] = ms.OCCUPIED
    return {'cells': cells[::-1].copy(), 'resolution': float(msg.info.resolution),
            'origin': (msg.info.origin.position.x, msg.info.origin.position.y, 0.0)}


def coverage(truth, mask, slam, spawn=(0.0, 0.0, 0.0)):
    """Share of `mask` cells that the SLAM map marks free."""
    seen = ms.resample_to_truth(truth, slam, spawn) == ms.FREE
    return float(np.count_nonzero(seen & mask) / np.count_nonzero(mask))


def time_to(curve, level):
    """First time at which coverage reaches `level`, or nan. curve is [(t, coverage)]."""
    for t, c in curve:
        if c >= level:
            return t
    return float('nan')
