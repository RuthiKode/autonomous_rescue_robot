#!/usr/bin/env python3
"""Does "hits / views" separate real survivors from look-alike obstacles?

  python3 scripts/hit_rate_diagnostic.py results/phase5all

For every run: the final /survivors tracks, the number of camera frames in which each track was
inside the camera's field of view and range (from the SLAM trajectory and the /detections
timestamps, occlusion ignored), and hits / views. Tracks within 1 m of a true survivor are 'true'.
"""
import csv
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

HALF_FOV = 0.6        # camera_hfov 1.2 rad
MIN_R, MAX_R = 0.4, 6.0
WORLDS = 'src/rescue_simulation/worlds'


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    mt = {k: get_message(v) for k, v in types.items() if k in ('/survivors', '/detections')}
    stamps, last = [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/detections':
            h = deserialize_message(data, mt[topic]).header.stamp
            stamps.append(h.sec + h.nanosec * 1e-9)
        elif topic == '/survivors':
            last = deserialize_message(data, mt[topic])
    return np.array(stamps), last


def main():
    root = sys.argv[1]
    for row in csv.DictReader(open(f'{root}/runs.csv')):
        bag = os.path.join(root, row['bag_path'])
        rd = os.path.join(root, row['run_dir']) if 'run_dir' in row else None
        run_dir = next((os.path.join(root, 'runs', d) for d in os.listdir(f'{root}/runs')
                        if d.startswith(row['run_id'][:-17]) and row['run_id'][-17:] in d), None) \
            or os.path.join(root, 'runs', row['run_id'])
        traj = np.loadtxt(f'{run_dir}/trajectory_slam.tum')       # t x y z qx qy qz qw
        stamps, msg = read(bag)
        t0 = stamps.min()
        # TUM times are sim seconds since the run start (same clock as the bag stamps)
        yaw = np.unwrap(2 * np.arctan2(traj[:, 6], traj[:, 7]))
        px = np.interp(stamps, traj[:, 0], traj[:, 1])
        py = np.interp(stamps, traj[:, 0], traj[:, 2])
        pyaw = np.interp(stamps, traj[:, 0], yaw)
        truth = yaml.safe_load(open(f"{WORLDS}/{row['world']}_survivors.yaml"))['survivors']
        t_xy = np.array([[s['position']['x'], s['position']['y']] for s in truth])
        print(f"{row['world']} seed {row['seed']}: frames {len(stamps)} (stamp0 {t0:.0f}, "
              f"traj {traj[0, 0]:.0f}-{traj[-1, 0]:.0f})")
        rows = []
        for s in msg.survivors:
            dx, dy = s.pose.position.x - px, s.pose.position.y - py
            r = np.hypot(dx, dy)
            bearing = (np.arctan2(dy, dx) - pyaw + math.pi) % (2 * math.pi) - math.pi
            views = int(np.count_nonzero((r > MIN_R) & (r < MAX_R) & (np.abs(bearing) < HALF_FOV)))
            true = np.hypot(*(t_xy - [s.pose.position.x, s.pose.position.y]).T).min() < 1.0
            rows.append((true, s.hit_count, views, s.confidence))
        for name, flag in (('true', True), ('false', False)):
            g = [(h, v, c) for t, h, v, c in rows if t == flag and v > 0]
            if g:
                rates = np.array([h / v for h, v, _ in g])
                print(f"  {name:<5} n={len(g):<3} hit rate median {np.median(rates):.2f}  "
                      f"min {rates.min():.2f}  max {rates.max():.2f}  "
                      f"| views median {np.median([v for _, v, _ in g]):.0f}")


if __name__ == '__main__':
    main()
