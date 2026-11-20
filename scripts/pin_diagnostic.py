#!/usr/bin/env python3
"""Why are there false pins? Final /survivors tracks of each run, split into true and false.

  python3 scripts/pin_diagnostic.py results/phase5 [method]

For the last /survivors message in every run's bag, tracks within 1 m of a true survivor are
"true", the rest "false". Prints hits and mean confidence of each group, and where the false
pins sit (distance to the nearest wall or obstacle is not computed; positions are printed).
"""
import csv
import glob
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

WORLDS = 'src/rescue_simulation/worlds'


def last_survivors(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type, last = get_message(types['/survivors']), None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/survivors':
            last = deserialize_message(data, msg_type)
    return last


def main():
    root = sys.argv[1]
    method = sys.argv[2] if len(sys.argv) > 2 else None
    for row in csv.DictReader(open(f'{root}/runs.csv')):
        if method and row['method'] != method:
            continue
        bag = os.path.join(root, row['bag_path'])
        if not os.path.isdir(bag):
            bag = glob.glob(f"{root}/{row['bag_path']}*")[0]
        msg = last_survivors(bag)
        truth = yaml.safe_load(open(f"{WORLDS}/{row['world']}_survivors.yaml"))['survivors']
        t_xy = np.array([[s['position']['x'], s['position']['y']] for s in truth])
        groups = {'true': [], 'false': []}
        for s in msg.survivors:
            d = np.hypot(*(t_xy - [s.pose.position.x, s.pose.position.y]).T).min()
            groups['true' if d < 1.0 else 'false'].append(s)
        print(f"{row['world']} seed {row['seed']}: {len(groups['true'])} true, "
              f"{len(groups['false'])} false tracks")
        for name, items in groups.items():
            if items:
                hits = [s.hit_count for s in items]
                conf = [s.confidence for s in items]
                print(f"  {name:<5} hits median {np.median(hits):.0f} (min {min(hits)}, "
                      f"max {max(hits)}), confidence median {np.median(conf):.2f} "
                      f"(min {min(conf):.2f}, max {max(conf):.2f})")


if __name__ == '__main__':
    main()
