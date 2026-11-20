"""Helpers for the synthetic survivor dataset (labels from a Gazebo segmentation camera).

Pure functions, so tests can run them without a simulator:
  boxes_from_mask   segmentation mask -> pixel boxes (one per visible survivor)
  assign_boxes      boxes -> the survivor each one shows (for the posture/occlusion breakdown)
  sample_poses      robot poses that look at a survivor, plus random free-space poses
  yolo_line         one YOLO-format label line
"""
import math

import numpy as np
from scipy import ndimage


def boxes_from_mask(mask, min_area=60, min_side=6, join_px=6):
    """Connected regions of a boolean mask as (x0, y0, x1, y1, area) pixel boxes (x1, y1 exclusive).

    Regions closer than `join_px` are one box: debris can cut a person into pieces, and the
    label should still be one person.
    """
    if not mask.any():
        return []
    joined = ndimage.binary_dilation(mask, iterations=join_px)
    labels, n = ndimage.label(joined)
    boxes = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero((labels == i) & mask)
        if ys.size < min_area:
            continue
        x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
        if x1 - x0 < min_side or y1 - y0 < min_side:
            continue
        boxes.append((int(x0), int(y0), int(x1), int(y1), int(ys.size)))
    return boxes


def yolo_line(box, width, height, cls=0):
    x0, y0, x1, y1 = box[:4]
    return (f'{cls} {(x0 + x1) / 2 / width:.6f} {(y0 + y1) / 2 / height:.6f} '
            f'{(x1 - x0) / width:.6f} {(y1 - y0) / height:.6f}')


def column_of(robot_x, robot_y, robot_yaw, x, y, fx, cx):
    """Image column of a world point seen by a camera that looks along the robot's heading.

    Returns (column, range in m, in front?). The camera sits a few cm from the robot origin,
    far below the pixel tolerance used for assignment.
    """
    dx, dy = x - robot_x, y - robot_y
    forward = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
    left = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
    if forward <= 0.05:
        return None, math.hypot(dx, dy), False
    return cx - fx * left / forward, math.hypot(dx, dy), True


def assign_boxes(boxes, robot_pose, survivors, fx, cx):
    """Index (into `survivors`) of the survivor each box shows, or -1.

    A box belongs to the nearest survivor whose predicted column lies within the box widened
    by 40 px.
    """
    x, y, yaw = robot_pose
    predicted = []
    for i, s in enumerate(survivors):
        col, rng, front = column_of(x, y, yaw, s['position']['x'], s['position']['y'], fx, cx)
        if front:
            predicted.append((i, col, rng))
    out = []
    for b in boxes:
        best, best_rng = -1, math.inf
        for i, col, rng in predicted:
            if b[0] - 40 <= col <= b[2] + 40 and rng < best_rng:
                best, best_rng = i, rng
        out.append(best)
    return out


def sample_poses(rng, free, origin, res, survivors, n, p_survivor=0.65,
                 min_range=1.2, max_range=6.0):
    """n robot poses (x, y, yaw, target_index or -1).

    `free` is a boolean grid (row 0 = max y, as the world grids use); a pose is kept only on a
    free cell. A survivor view stands 1.2-6 m away and looks at the survivor with about 0.25 rad
    of jitter, so the survivor is not always centred; the rest are random free poses with random
    headings, which supply the frames without anyone in view.
    """
    height = free.shape[0]

    def is_free(x, y):
        c = int((x - origin[0]) / res)
        r = height - 1 - int((y - origin[1]) / res)
        return 0 <= r < free.shape[0] and 0 <= c < free.shape[1] and bool(free[r, c])

    poses, tries = [], 0
    while len(poses) < n and tries < n * 200:
        tries += 1
        if survivors and rng.random() < p_survivor:
            k = int(rng.integers(len(survivors)))
            sx, sy = survivors[k]['position']['x'], survivors[k]['position']['y']
            d, bearing = rng.uniform(min_range, max_range), rng.uniform(-math.pi, math.pi)
            x, y = sx + d * math.cos(bearing), sy + d * math.sin(bearing)
            yaw = math.atan2(sy - y, sx - x) + rng.normal(0.0, 0.25)
            target = k
        else:
            rows, cols = np.nonzero(free)
            j = int(rng.integers(rows.size))
            x = origin[0] + (cols[j] + 0.5) * res
            y = origin[1] + (height - 1 - rows[j] + 0.5) * res
            yaw, target = rng.uniform(-math.pi, math.pi), -1
        if is_free(x, y):
            poses.append((x, y, yaw, target))
    return poses
