#!/usr/bin/env python3
"""Count header-stamp gaps of topics in a rosbag: bag_stamp_gaps.py <bag> <topic> [...]"""
import struct
import sys

import rosbag2_py

from rescue_eval.camera_rate_check import analyse

bag, topics = sys.argv[1], sys.argv[2:]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
            rosbag2_py.ConverterOptions('', ''))
stamps = {t: [] for t in topics}
walls = {t: [] for t in topics}
while reader.has_next():
    topic, data, received_ns = reader.read_next()
    if topic in stamps:
        sec, nsec = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
        stamps[topic].append(sec + nsec * 1e-9)
        walls[topic].append(received_ns * 1e-9)
for t in topics:
    s = analyse(stamps[t], walls[t])
    if 'dropped' not in s:
        print(f'{t}: only {s["received"]} messages in the bag')
        continue
    print(f'{t}: recorded {s["received"]}, dropped {s["dropped"]} ({s["drop_pct"]:.1f}%),'
          f' longest gap {s["max_gap_frames"]} frames, nominal {s["nominal_hz"]:.2f} Hz,'
          f' sim-time rate {s["sim_hz"]:.2f} Hz, wall rate {s["wall_hz"]:.2f} Hz')
