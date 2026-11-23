#!/usr/bin/env python3
"""Render a mission from its rosbag as a video: map growing, robot path, pins, mission state.

  python3 scripts/render_demo.py results/sweep <run_id> docs/media/demo.mp4 [--fps 12] [--step 1.0]

This is an offline replay of recorded data (/map, /survivors, /mission/state from the bag, the SLAM
trajectory from the run directory, the true survivors from the world files). It is not a screen
recording of Foxglove: that needs the Foxglove Studio window and is made by opening
foxglove/rescue_layout.json while the mission runs (docs/FOXGLOVE.md).
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

PIN_COLOUR = {'candidate': '#ffcc00', 'confirmed': '#ff3b30', 'verified': '#34c759',
              'rejected': '#555555', 'merged': '#999999'}


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in ('/map', '/survivors', '/mission/state')}
    events = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in types:
            events.append((t_ns * 1e-9, topic, deserialize_message(data, types[topic])))
    return events


def grid_image(msg):
    a = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    img = np.full(a.shape, 0.55)                    # unknown: mid grey
    img[(a >= 0) & (a <= 25)] = 1.0                 # free: white
    img[a >= 65] = 0.0                              # occupied: black
    ox, oy, res = msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution
    return img, (ox, ox + a.shape[1] * res, oy, oy + a.shape[0] * res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('results')
    ap.add_argument('run_id')
    ap.add_argument('out')
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--step', type=float, default=1.0, help='sim seconds between frames')
    ap.add_argument('--worlds', default='src/rescue_simulation/worlds')
    args = ap.parse_args()

    import imageio.v2 as imageio
    run_dir = f'{args.results}/runs/{args.run_id}'
    world = args.run_id.split('_')[0] if not args.run_id.startswith('open_plan') else 'open_plan'
    truth = yaml.safe_load(open(f'{args.worlds}/{world}_survivors.yaml'))['survivors']
    traj = np.loadtxt(f'{run_dir}/trajectory_slam.tum')
    cov = [(float(r['sim_time_s']), float(r['coverage']))
           for r in csv.DictReader(open(f'{run_dir}/coverage.csv'))]
    events = read_bag(f'{args.results}/bags/{args.run_id}')
    t_begin = events[0][0]
    t_end = events[-1][0]
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    state_map, survivors, mission, i = None, [], 'STARTING', 0
    writer = imageio.get_writer(args.out, fps=args.fps, codec='libx264', quality=7,
                                macro_block_size=2)
    t = t_begin
    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=100)
    while t <= t_end:
        while i < len(events) and events[i][0] <= t:
            _, topic, msg = events[i]
            if topic == '/map':
                state_map = msg
            elif topic == '/survivors':
                survivors = list(msg.survivors)
            else:
                mission = msg.state
            i += 1
        ax.clear()
        if state_map is not None:
            img, extent = grid_image(state_map)
            ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', extent=extent)
        sel = traj[:, 0] - traj[0, 0] <= t - t_begin + 30     # TUM stamps are sim time since start
        ax.plot(traj[sel, 1], traj[sel, 2], color='#0a84ff', lw=1.2)
        if sel.any():
            ax.plot(traj[sel, 1][-1], traj[sel, 2][-1], 'o', color='#ff9f0a', ms=7)
        ax.scatter([s['position']['x'] for s in truth], [s['position']['y'] for s in truth],
                   s=220, facecolors='none', edgecolors='#30d158', linewidths=2, label='true survivor')
        for s in survivors:
            ax.scatter([s.pose.position.x], [s.pose.position.y], s=70, marker='X',
                       color=PIN_COLOUR.get(s.state, 'k'), zorder=5)
        c = [v for tt, v in cov if tt <= t - t_begin]
        ax.set_title(f'{world}   t = {t - t_begin:5.0f} s   state {mission}   coverage '
                     f'{(c[-1] if c else 0):.2f}', fontsize=11)
        ax.set_aspect('equal')
        ax.set_xticks([]), ax.set_yticks([])
        fig.canvas.draw()
        writer.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3])
        t += args.step
    writer.close()
    print('wrote', args.out)


if __name__ == '__main__':
    main()
