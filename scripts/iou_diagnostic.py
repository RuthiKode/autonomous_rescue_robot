"""Why is IoU low? Strict IoU against IoU with a 1 and 2 cell tolerance, and cell counts.

  python3 scripts/iou_diagnostic.py results/phase2_slip

Tolerance IoU is rescue_eval.map_score.tolerance_iou (the map_iou_tol metric for k = 1).
"""
import csv
import glob
import os
import sys

import numpy as np
from scipy import ndimage

from rescue_eval import map_score as ms

out = sys.argv[1]
from ament_index_python.packages import get_package_share_directory
sim = os.path.join(get_package_share_directory('rescue_simulation'), 'worlds')
rows = [r for r in csv.DictReader(open(os.path.join(out, 'runs.csv')))
        if r['status'] == 'ok' and r['method'] == 'slam_replay_slip']
print(f"{'run':<10}{'IoU':>6}{'tol1':>6}{'tol2':>6}{'truthOcc':>9}{'slamOcc':>8}{'slam/truth':>11}"
      f"{'miss%':>7}{'extra%':>7}{'median dist of slam-only cells (cells)':>40}")
for r in rows:
    truth = ms.load_map(f"{sim}/{r['world']}_truth_map.yaml")
    slam = ms.load_map(os.path.join(out, r['map_path']))
    t = truth['cells']
    s = ms.resample_to_truth(truth, slam, (0.0, 0.0, 0.0))
    known = (t != ms.UNKNOWN) & (s != ms.UNKNOWN)
    to, so = (t == ms.OCCUPIED) & known, (s == ms.OCCUPIED) & known
    iou = (to & so).sum() / (to | so).sum()

    def tol(k):
        return ms.tolerance_iou(to, so, k)
    dist_to_truth = ndimage.distance_transform_edt(~to)
    only_slam = so & ~to
    only_truth = to & ~so
    med = np.median(dist_to_truth[only_slam]) if only_slam.any() else float('nan')
    print(f"{r['world'][:9]:<10}{iou:6.3f}{tol(1):6.3f}{tol(2):6.3f}{to.sum():9d}{so.sum():8d}"
          f"{so.sum() / to.sum():11.2f}{100 * only_truth.sum() / to.sum():7.1f}"
          f"{100 * only_slam.sum() / so.sum():7.1f}{med:40.1f}")
