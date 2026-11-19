#!/usr/bin/env python3
"""Count long-range pose-graph constraints in saved SLAM runs.

  python3 scripts/loop_closure_check.py results/phase2_slip [--min-gap 20]

slam_toolbox publishes its pose graph as nodes (sphere markers, one per scan) and
one LINE_LIST of edges. An edge joining two nodes whose ids differ by more than
--min-gap links scans that are far apart in time but close in space. That is
what a loop closure (or a re-visit of a mapped area) produces. Edge counts show
that such constraints exist; they do not show that each one is correct.

Per run it also prints the SLAM-to-truth position error just before and just
after the first sample in which the graph holds a long-range edge, read from the
saved trajectories. The graph is read once per second of the bag.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from visualization_msgs.msg import MarkerArray

TOPIC = '/slam_toolbox/graph_visualization'


def long_range_edges(msg, min_gap):
    """Return (nodes, total_edges, long_range_edges) for one MarkerArray."""
    nodes = {m.id: (m.pose.position.x, m.pose.position.y) for m in msg.markers
             if m.ns == 'slam_toolbox' and m.action == 0}
    if not nodes:
        return 0, 0, 0
    ids = np.array(sorted(nodes))
    xy = np.array([nodes[i] for i in ids])
    edges = [m for m in msg.markers if m.ns == 'slam_toolbox_edges' and m.action == 0
             for m in [m]]
    total = long = 0
    for m in edges:
        pts = m.points
        for a, b in zip(pts[0::2], pts[1::2]):
            ia = ids[np.argmin(np.hypot(xy[:, 0] - a.x, xy[:, 1] - a.y))]
            ib = ids[np.argmin(np.hypot(xy[:, 0] - b.x, xy[:, 1] - b.y))]
            total += 1
            long += abs(int(ia) - int(ib)) > min_gap
    return len(nodes), total, long


def scan_bag(bag, min_gap, every_s=1.0):
    """Yield (bag time in seconds, nodes, edges, long-range edges), once per every_s."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    t0 = next_t = None
    while reader.has_next():
        topic, data, ns = reader.read_next()
        if topic != TOPIC:
            continue
        t = ns * 1e-9
        t0 = t if t0 is None else t0
        if next_t is None or t >= next_t:
            next_t = t + every_s
            yield (t - t0,) + long_range_edges(deserialize_message(data, MarkerArray), min_gap)


def error_around(run_dir, bag_time_s):
    """SLAM-to-truth position error 5 s before and 5 s after a time since the drive started."""
    t = np.loadtxt(os.path.join(run_dir, 'trajectory_truth.tum'))
    e = np.loadtxt(os.path.join(run_dir, 'trajectory_slam.tum'))
    err = np.hypot(t[:, 1] - e[:, 1], t[:, 2] - e[:, 2])
    rel = t[:, 0] - t[0, 0]
    def at(x):
        return float(err[int(np.argmin(np.abs(rel - x)))])
    return at(max(0.0, bag_time_s - 5.0)), at(bag_time_s + 5.0)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('results_dir')
    p.add_argument('--min-gap', type=int, default=20, help='node-id gap that counts as long-range')
    p.add_argument('--method', default=None, help='only rows with this method label')
    args = p.parse_args()
    with open(os.path.join(args.results_dir, 'runs.csv'), newline='') as f:
        rows = [r for r in csv.DictReader(f) if r['status'] == 'ok' and r['map_path']
                and args.method in (None, r['method'])]
    print(f"{'run':<34}{'nodes':>6}{'edges':>6}{'long':>6}  first long-range edge at"
          f"  error 5 s before -> 5 s after (m)")
    for r in rows:
        bag = os.path.join(args.results_dir, r['bag_path'])
        run_dir = os.path.join(args.results_dir, os.path.dirname(r['map_path']))
        samples = list(scan_bag(bag, args.min_gap))
        nodes, edges, long = samples[-1][1:]
        first = next((s[0] for s in samples if s[3] > 0), None)
        last = max((b[0] for a, b in zip(samples, samples[1:]) if b[3] > a[3]), default=None)
        # bag time starts at recording start, the trajectory file at the first sample
        tail = ''
        if first is not None:
            before, after = error_around(run_dir, first)
            tail = f'{first:6.1f} s  {before:.3f}->{after:.3f}'
        if last is not None:
            before, after = error_around(run_dir, last)
            tail += f'   last increase {last:6.1f} s  {before:.3f}->{after:.3f}'
        print(f"{r['run_id'][:33]:<34}{nodes:>6}{edges:>6}{long:>6}  {tail or 'none':<24}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
