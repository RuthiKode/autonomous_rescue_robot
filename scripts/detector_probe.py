#!/usr/bin/env python3
"""Probe the pretrained detector on the low-poly survivors.

  python3 scripts/detector_probe.py <world> [--out DIR]

Needs `ros2 launch rescue_bringup sim.launch.py world:=<world> headless:=true` running. For
every survivor and a few distances the robot is teleported to a free spot in front of the
survivor, looking at it; one RGB frame is saved and the YOLO model (in ~/venvs/yolo) runs on
it. Prints one line per view: survivor, posture, distance, and what the detector found.
Annotated frames are written to --out.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

DISTANCES = (1.5, 2.5, 4.0)
YOLO_SNIPPET = r'''
import json, sys
from ultralytics import YOLO
m = YOLO(sys.argv[1])
out = {}
for path in sys.argv[2:]:
    r = m.predict(path, imgsz=640, conf=0.05, classes=[0], device="cuda:0", verbose=False)[0]
    out[path] = [[float(c), [float(v) for v in b]] for b, c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())]
    r.save(path.replace(".png", "_det.png"))
print(json.dumps(out))
'''


def set_pose(world, x, y, yaw):
    req = (f'name: "rescue_robot", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.05}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
    subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                   check=True, capture_output=True)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('world')
    p.add_argument('--out', default='results/detector_probe')
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    from ament_index_python.packages import get_package_share_directory
    from rescue_eval.auto_drive import Planner
    from rescue_eval.nav_goals import make_planner
    worlds = os.path.join(get_package_share_directory('rescue_simulation'), 'worlds')
    truth = yaml.safe_load(open(os.path.join(worlds, args.world + '_survivors.yaml')))
    planner = make_planner(args.world, 0.30)          # free = the robot body fits here

    rclpy.init()
    node = rclpy.create_node('detector_probe')
    latest = {}
    node.create_subscription(Image, '/camera/image_raw', lambda m: latest.__setitem__('rgb', m),
                             rclpy.qos.QoSProfile(depth=2, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
    node.create_subscription(Image, '/camera/depth', lambda m: latest.__setitem__('depth', m),
                             rclpy.qos.QoSProfile(depth=2, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
    views = []
    for s in truth['survivors']:
        sx, sy = s['position']['x'], s['position']['y']
        for d in DISTANCES:
            for bearing in (0.0, math.pi / 2, math.pi, -math.pi / 2):     # try four approach sides
                cx, cy = sx + d * math.cos(bearing), sy + d * math.sin(bearing)
                r, c = planner.cell(cx, cy)
                if 0 <= r < planner.free.shape[0] and 0 <= c < planner.free.shape[1] \
                        and planner.free[r, c] and planner.reachable()[r, c]:
                    break
            else:
                continue
            yaw = math.atan2(sy - cy, sx - cx)
            set_pose(args.world, cx - 0.13 * math.cos(yaw), cy - 0.13 * math.sin(yaw), yaw)
            latest.clear()
            t_end = time.monotonic() + 6.0
            time.sleep(1.0)                                   # let the physics settle
            latest.clear()
            while time.monotonic() < t_end and len(latest) < 2:
                rclpy.spin_once(node, timeout_sec=0.1)
            if 'rgb' not in latest:
                continue
            m = latest['rgb']
            rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            from PIL import Image as PILImage
            path = os.path.join(args.out, f"{args.world}_s{s['id']}_{s['posture']}_{d:.1f}m.png")
            PILImage.fromarray(rgb).save(path)
            depth = None
            if 'depth' in latest:
                dm = latest['depth']
                arr = np.frombuffer(dm.data, np.float32).reshape(dm.height, dm.step // 4)[:, :dm.width]
                depth = float(np.nanmedian(arr[int(dm.height * 0.35):int(dm.height * 0.65),
                                               int(dm.width * 0.4):int(dm.width * 0.6)]))
            views.append((s, d, path, depth))
    node.destroy_node()
    rclpy.try_shutdown()

    weights = os.path.expanduser('~/venvs/yolo/weights/yolov8n.pt')
    res = subprocess.run([os.path.expanduser('~/venvs/yolo/bin/python'), '-c', YOLO_SNIPPET, weights]
                         + [v[2] for v in views], capture_output=True, text=True)
    dets = json.loads(res.stdout.strip().splitlines()[-1])
    found = total = 0
    print(f"{'survivor':<28}{'dist':>5}{'centre depth':>13}  detections (conf >= 0.05)")
    for s, d, path, depth in views:
        det = dets[path]
        total += 1
        found += bool(det)
        best = max(det, key=lambda x: x[0]) if det else None
        label = f"#{s['id']} {s['posture']}{' (occluded)' if s['occluded'] else ''}"
        print(f"{label:<28}{d:>5.1f}{depth if depth is not None else float('nan'):>13.2f}  "
              + (f"{len(det)} box(es), best conf {best[0]:.2f}" if det else 'none'))
    print(f'views with at least one person box: {found} of {total}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
