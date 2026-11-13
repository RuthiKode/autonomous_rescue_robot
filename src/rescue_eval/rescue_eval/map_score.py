"""Score a SLAM occupancy map against the generator's ground-truth grid.

  ros2 run rescue_eval map_score --truth <world>_truth_map.yaml --map run/map.yaml

Alignment. No registration is fitted. The ground-truth grid is in the world
frame. slam_toolbox's map frame starts where the robot starts: the first scan is
inserted at the odom-frame origin, which is the robot's start pose. So the map
frame sits at the robot spawn pose in the world, and the spawn pose from the
world's ground-truth YAML is the whole transform:

    p_world = R(spawn_yaw) * p_map + (spawn_x, spawn_y)

(The generator spawns the robot at the world origin facing +x, so this is the
identity today; the code does not rely on that.) Each truth cell centre is
mapped into the SLAM map and takes the value of the SLAM cell it falls in.
Any drift or rotation of the SLAM map therefore lowers the score instead of
being hidden by a best-fit alignment.

Scores, both over cells that are known (free or occupied) in both maps:
  IoU       |truth occupied AND slam occupied| / |truth occupied OR slam occupied|
  accuracy  share of those cells where the two maps agree
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image
import yaml

OCCUPIED, FREE, UNKNOWN = 100, 0, -1


def load_map(yaml_path):
    """Load a map_server map. Returns dict(cells, resolution, origin).

    cells is an int8 array with row 0 at the top, values OCCUPIED / FREE / UNKNOWN.
    """
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_path)
    pixels = np.asarray(Image.open(image_path).convert('L'), dtype=np.float64)
    occ_prob = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[occ_prob > meta['occupied_thresh']] = OCCUPIED
    cells[occ_prob < meta['free_thresh']] = FREE
    # map_saver_cli writes unknown cells as grey 205 together with free_thresh 0.25.
    # 205 is an occupancy of 0.196, so the thresholds alone would read unknown as
    # free. Grey 205 is therefore always treated as unknown.
    cells[pixels == 205] = UNKNOWN
    return {'cells': cells, 'resolution': float(meta['resolution']),
            'origin': tuple(float(v) for v in meta['origin'][:3])}


def resample_to_truth(truth, slam, spawn):
    """Return the SLAM map sampled at every truth cell centre (same shape as truth)."""
    h, w = truth['cells'].shape
    res = truth['resolution']
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    wx = truth['origin'][0] + (cols + 0.5) * res
    wy = truth['origin'][1] + (h - 1 - rows + 0.5) * res
    # world -> map: inverse of p_world = R(yaw) p_map + t
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    dx, dy = wx - spawn[0], wy - spawn[1]
    mx, my = c * dx + s * dy, -s * dx + c * dy
    # map -> SLAM image indices (SLAM map origin yaw is 0 for slam_toolbox)
    sh, sw = slam['cells'].shape
    sc = np.floor((mx - slam['origin'][0]) / slam['resolution']).astype(int)
    sr = sh - 1 - np.floor((my - slam['origin'][1]) / slam['resolution']).astype(int)
    inside = (sc >= 0) & (sc < sw) & (sr >= 0) & (sr < sh)
    out = np.full((h, w), UNKNOWN, dtype=np.int8)
    out[inside] = slam['cells'][sr[inside], sc[inside]]
    return out


def score(truth, slam, spawn=(0.0, 0.0, 0.0)):
    """Return dict(iou, accuracy, known_cells, truth_occupied, slam_occupied)."""
    t = truth['cells']
    s = resample_to_truth(truth, slam, spawn)
    known = (t != UNKNOWN) & (s != UNKNOWN)
    t_occ, s_occ = (t == OCCUPIED) & known, (s == OCCUPIED) & known
    union = int(np.count_nonzero(t_occ | s_occ))
    n_known = int(np.count_nonzero(known))
    return {
        'iou': float(np.count_nonzero(t_occ & s_occ) / union) if union else float('nan'),
        'accuracy': float(np.count_nonzero((t == s) & known) / n_known) if n_known
        else float('nan'),
        'known_cells': n_known,
        'truth_occupied': int(np.count_nonzero(t_occ)),
        'slam_occupied': int(np.count_nonzero(s_occ)),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--truth', required=True, help='<world>_truth_map.yaml')
    parser.add_argument('--map', required=True, help='SLAM map yaml written by map_saver_cli')
    parser.add_argument('--spawn', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=('X', 'Y', 'YAW'), help='robot start pose in the world frame')
    args = parser.parse_args(argv)
    result = score(load_map(args.truth), load_map(args.map), tuple(args.spawn))
    for key, value in result.items():
        print(f'{key}: {value:.4f}' if isinstance(value, float) else f'{key}: {value}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
