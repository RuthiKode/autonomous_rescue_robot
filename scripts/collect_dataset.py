#!/usr/bin/env python3
"""Collect labelled frames for one generated world (layout + seed), no hand labelling.

  python3 scripts/collect_dataset.py --layout rooms --seed 3 --frames 100 --out results/dataset

Generates the world, starts the simulator headless with the segmentation camera, teleports the
robot to sampled poses (65 % looking at a survivor, the rest random free poses), changes the sun's
direction and strength every few frames, and for each pose saves the RGB frame and a YOLO label
file made from the segmentation mask (label 1 = survivor). meta.csv holds the pose, the light
and, for every box, which survivor it shows (posture, occluded).

Needs the workspace sourced. One world per call; scripts/collect_all.sh loops over worlds.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
import zlib

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy import ndimage
from sensor_msgs.msg import Image
import yaml

from rescue_perception.dataset import assign_boxes, boxes_from_mask, sample_poses, yolo_line
from rescue_simulation import generate_worlds as gw

HFOV, WIDTH = 1.2, 640                       # robot_params.xacro: camera_hfov, camera_width
FX = (WIDTH / 2) / math.tan(HFOV / 2)
CX = WIDTH / 2
ROBOT_CLEARANCE_M = 0.38                     # cells nearer a wall than this are not used
META_COLUMNS = ['image', 'layout', 'seed', 'frame', 'x', 'y', 'yaw', 'target', 'sun_elevation',
                'sun_strength', 'n_boxes', 'boxes']


def gz_service(world, service, reqtype, req, timeout_ms=3000):
    return subprocess.run(['gz', 'service', '-s', f'/world/{world}/{service}', '--reqtype',
                           reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms),
                           '--req', req], capture_output=True, text=True)


def set_pose(world, x, y, yaw):
    return gz_service(world, 'set_pose', 'gz.msgs.Pose',
                      f'name: "rescue_robot", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.05}}, '
                      f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')


def set_sun(world, elevation, azimuth, strength):
    dx = math.cos(elevation) * math.cos(azimuth)
    dy = math.cos(elevation) * math.sin(azimuth)
    dz = -math.sin(elevation)
    s = strength
    return gz_service(world, 'light_config', 'gz.msgs.Light',
                      f'name: "sun", type: DIRECTIONAL, cast_shadows: true, '
                      f'diffuse: {{r: {s:.3f}, g: {s:.3f}, b: {s:.3f}, a: 1}}, '
                      f'specular: {{r: 0.2, g: 0.2, b: 0.2, a: 1}}, '
                      f'direction: {{x: {dx:.4f}, y: {dy:.4f}, z: {dz:.4f}}}')


def as_array(msg, channels):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return a[:, :msg.width * channels].reshape(msg.height, msg.width, channels)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--layout', required=True, choices=['rooms', 'collapsed', 'open_plan'])
    ap.add_argument('--seed', type=int, required=True, help='world seed (42 = the committed worlds)')
    ap.add_argument('--frames', type=int, default=100)
    ap.add_argument('--out', default='results/dataset')
    ap.add_argument('--light-every', type=int, default=10, help='frames between sun changes')
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed * 1000 + zlib.crc32(args.layout.encode()) % 997)
    tag = f'{args.layout}_s{args.seed}'
    wdir = os.path.join(args.out, 'worlds', f's{args.seed}')
    os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(args.out, 'labels'), exist_ok=True)
    gw.main(['--output-dir', wdir, '--world', args.layout, '--seed', str(args.seed)])

    cfg = gw.load_config()
    builder = gw.build(args.layout, dict(cfg['worlds'][args.layout]), cfg['common'], args.seed)
    grid, origin = gw.occupancy_grid(builder, 0.05, 0.5, 0.0, 0.4)
    clearance = ndimage.distance_transform_edt(grid == gw.GRID_FREE) * 0.05
    free = clearance > ROBOT_CLEARANCE_M
    survivors = [{'position': {'x': s['x'], 'y': s['y']}, 'posture': s['posture'],
                  'occluded': bool(s['occluded'])} for s in builder.survivors]
    poses = sample_poses(rng, free, origin, 0.05, survivors, args.frames)

    log = open(os.path.join(args.out, f'sim_{tag}.log'), 'w')
    sim = subprocess.Popen(
        ['ros2', 'launch', 'rescue_bringup', 'sim.launch.py', f'world:={args.layout}',
         'headless:=true', 'seg_camera:=true', 'ekf:=false', f'worlds_dir:={os.path.abspath(wdir)}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('collect_dataset')
    latest = {'rgb': [], 'seg': []}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

    def keep(kind):
        def cb(m):
            latest[kind].append(m)
            del latest[kind][:-6]
        return cb
    node.create_subscription(Image, '/camera/image_raw', keep('rgb'), qos)
    node.create_subscription(Image, '/camera/segmentation/labels_map', keep('seg'), qos)

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def grab_pair(timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            for s in reversed(latest['seg']):
                for r in reversed(latest['rgb']):
                    if abs(stamp(r) - stamp(s)) < 0.03:
                        return r, s
        return None

    rows, saved, ok = [], 0, False
    try:
        for _ in range(120):                       # wait for the simulator and both cameras
            rclpy.spin_once(node, timeout_sec=0.5)
            if latest['rgb'] and latest['seg']:
                ok = True
                break
        if not ok:
            sys.exit('simulator did not start; see ' + log.name)
        sun = (0.9, 0.0, 0.8)
        for k, (x, y, yaw, target) in enumerate(poses):
            if k % args.light_every == 0:
                sun = (float(rng.uniform(0.45, 1.35)), float(rng.uniform(-math.pi, math.pi)),
                       float(rng.uniform(0.35, 1.0)))
                set_sun(args.layout, *sun)
            set_pose(args.layout, x, y, yaw)
            time.sleep(0.7)
            latest['rgb'].clear(), latest['seg'].clear()
            pair = grab_pair(5.0)
            if pair is None:
                continue
            rgb_msg, seg_msg = pair
            seg = as_array(seg_msg, 3)[:, :, 0]
            boxes = boxes_from_mask(seg == 1)
            who = assign_boxes(boxes, (x, y, yaw), survivors, FX, CX)
            name = f'{tag}_{k:04d}'
            PILImage.fromarray(as_array(rgb_msg, 3)).save(
                os.path.join(args.out, 'images', name + '.png'))
            with open(os.path.join(args.out, 'labels', name + '.txt'), 'w') as f:
                f.write(''.join(yolo_line(b, rgb_msg.width, rgb_msg.height) + '\n' for b in boxes))
            rows.append({
                'image': name, 'layout': args.layout, 'seed': args.seed, 'frame': k,
                'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3), 'target': target,
                'sun_elevation': round(sun[0], 3), 'sun_strength': round(sun[2], 3),
                'n_boxes': len(boxes),
                'boxes': json.dumps([
                    [*b, survivors[i]['posture'] if i >= 0 else None,
                     survivors[i]['occluded'] if i >= 0 else None]
                    for b, i in zip(boxes, who)])})
            saved += 1
            if saved % 25 == 0:
                print(f'{tag}: {saved} frames', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            os.killpg(sim.pid, signal.SIGINT)
            sim.wait(timeout=25)
        except Exception:
            os.killpg(sim.pid, signal.SIGKILL)
        subprocess.run(['bash', os.path.expanduser('~/cleanup.sh')], capture_output=True)

    path = os.path.join(args.out, 'meta.csv')
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=META_COLUMNS)
        if new:
            w.writeheader()
        w.writerows(rows)
    with_box = sum(1 for r in rows if r['n_boxes'])
    print(f'{tag}: {saved} frames saved, {with_box} with at least one survivor box')
    return 0


if __name__ == '__main__':
    sys.exit(main())
