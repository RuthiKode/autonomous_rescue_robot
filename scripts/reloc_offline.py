#!/usr/bin/env python3
"""Offline checks for global relocalization, on recorded failure-test bags.

  python3 scripts/reloc_offline.py landing   results/failure_tests      # go/no-go: was the landing spot mapped?
  python3 scripts/reloc_offline.py calibrate OUT_DIR BAG_DIR...          # detection scores and global search vs truth

For each teleport run: the /map just before the teleport, and the true landing pose (world frame,
from trajectory_truth.tum) moved into the map frame with the spawn pose. Prints whether the landing
cell was known free in that map, and how much of a 2 m disc around it was known.
"""
import json
import math
import os
import sys

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def read_topics(bag_dir, topics):
    """Yield (topic, sim time s, msg) for the given topics."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in classes:
            yield topic, stamp * 1e-9, deserialize_message(data, classes[topic])


def grid(msg):
    """OccupancyGrid -> (int8 array [row=y, col=x], resolution, (origin x, origin y))."""
    a = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return a, msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y)


def world_to_map(p, spawn):
    dx, dy = p[0] - spawn[0], p[1] - spawn[1]
    c, s = math.cos(-spawn[2]), math.sin(-spawn[2])
    return c * dx - s * dy, s * dx + c * dy, p[2] - spawn[2]


def load_tum(path):
    rows = np.loadtxt(path)
    yaw = 2 * np.arctan2(rows[:, 6], rows[:, 7])
    return rows[:, 0], np.stack([rows[:, 1], rows[:, 2], yaw], axis=1)


def spawn_of(world):
    path = os.path.join(os.path.expanduser('~/autonomous_rescue_robot/install/rescue_simulation/share'),
                        'rescue_simulation', 'worlds', f'{world}_survivors.yaml')
    if not os.path.exists(path):
        path = os.path.join('src/rescue_simulation/worlds', f'{world}_survivors.yaml')
    with open(path) as f:
        t = yaml.safe_load(f)
    s = t['robot_spawn']
    return s['x'], s['y'], s['yaw']


def teleport_runs(results_dir):
    runs = os.path.join(results_dir, 'runs')
    for rid in sorted(os.listdir(runs)):
        inj = os.path.join(runs, rid, 'injection.json')
        if 'teleport' in rid and os.path.exists(inj):
            with open(inj) as f:
                yield rid, os.path.join(runs, rid), os.path.join(results_dir, 'bags', rid), json.load(f)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def find_teleport(bag_dir):
    """(sim time of the jump, true pose after it (world frame), [(t, /map msg)]) from the bag itself."""
    prev, t_jump, land, maps = None, None, None, []
    for topic, t, msg in read_topics(bag_dir, ['/ground_truth/odom', '/map']):
        if topic == '/map':
            maps.append((t, msg))
            continue
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw_of(p.orientation))
        if t_jump is None and prev is not None and math.hypot(cur[0] - prev[0], cur[1] - prev[1]) > 1.0:
            t_jump, land = t, cur
        prev = cur
    return t_jump, land, maps


def landing(results_dir):
    for rid, run_dir, bag_dir, inj in teleport_runs(results_dir):
        world = rid.split('_')[0]
        spawn = spawn_of(world)
        t_jump, land_w, maps = find_teleport(bag_dir)
        last_map = [m for m in maps if m[0] < t_jump][-1]
        land = world_to_map(land_w, spawn)
        a, res, org = grid(last_map[1])
        cx, cy = int((land[0] - org[0]) / res), int((land[1] - org[1]) / res)
        inside = 0 <= cx < a.shape[1] and 0 <= cy < a.shape[0]
        cell = int(a[cy, cx]) if inside else None
        r = int(2.0 / res)
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disc = (xx ** 2 + yy ** 2) <= r * r
        known = 0.0
        if inside:
            pad = np.pad(a, r, constant_values=-1)
            win = pad[cy:cy + 2 * r + 1, cx:cx + 2 * r + 1]
            known = float(np.mean(win[disc] >= 0))
        print(f'{rid}: map at {last_map[0]:.1f} s, landing (map frame) ({land[0]:.2f}, {land[1]:.2f}), '
              f'cell {cell} ({"free" if cell == 0 else "unknown" if cell == -1 else "occupied/outside"}), '
              f'known within 2 m {known:.2f}')


def _tf2d(t):
    return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def replay(bag_dir, every_s=0.5, with_odom=False):
    """Yield (t, field, scan points in base frame, SLAM pose, true pose in world frame) about every_s,
    plus the odom -> base_link pose when with_odom."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rescue_mission'))
    from rescue_mission.relocalize import Field, scan_points
    tfs, static, field, truth, last = {}, {}, None, None, -1e9
    topics = ['/tf', '/tf_static', '/map', '/scan', '/ground_truth/odom']
    for topic, t, msg in read_topics(bag_dir, topics):
        if topic in ('/tf', '/tf_static'):
            for tr in msg.transforms:
                (static if topic == '/tf_static' else tfs)[(tr.header.frame_id, tr.child_frame_id)] = _tf2d(tr)
        elif topic == '/map':
            field = Field.from_msg(msg)
        elif topic == '/ground_truth/odom':
            p = msg.pose.pose
            truth = (p.position.x, p.position.y, yaw_of(p.orientation))
        elif topic == '/scan' and t - last >= every_s and field is not None and truth is not None:
            if ('map', 'odom') not in tfs or ('odom', 'base_link') not in tfs:
                continue
            sensor = _sensor_pose(static, msg.header.frame_id)
            if sensor is None:
                continue
            last = t
            pts = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                              sensor)
            slam = _compose(tfs[('map', 'odom')], tfs[('odom', 'base_link')])
            if with_odom:
                yield t, field, pts, slam, truth, tfs[('odom', 'base_link')]
            else:
                yield t, field, pts, slam, truth


def _sensor_pose(static, frame):
    """base_link -> frame from the static transforms (walks up the chain)."""
    pose, f = (0.0, 0.0, 0.0), frame
    for _ in range(6):
        if f == 'base_link':
            return pose
        parent = next((p for (p, c) in static if c == f), None)
        if parent is None:
            return None
        pose = _compose(static[(parent, f)], pose)
        f = parent
    return None


def calibrate(out_dir, *bag_dirs):
    """Scores at the SLAM pose: normal driving vs after the teleport; global search vs truth."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rescue_mission'))
    from rescue_mission.relocalize import KidnapDetector, global_search, score
    import time as _t
    os.makedirs(out_dir, exist_ok=True)
    rows, searches = [], []
    for bag in bag_dirs:
        rid = os.path.basename(bag.rstrip('/'))
        world = rid.split('_fail')[0].split('_own')[0].split('_lite')[0]
        world = 'open_plan' if rid.startswith('open_plan') else world
        spawn = spawn_of(world)
        t_jump = None
        if 'teleport' in rid:
            t_jump, _, _ = find_teleport(bag)
        det = KidnapDetector()
        searched = False
        for t, field, pts, slam, truth in replay(bag):
            s, n = score(field, pts, slam)
            phase = 'normal' if t_jump is None or t < t_jump else ('after' if t < t_jump + 12 else 'late')
            kid, _, _ = det.update(field, pts, slam)
            rows.append((rid, world, round(t, 1), phase, round(s, 3), n, int(kid)))
            if t_jump is not None and not searched and t >= t_jump + 3.0:
                searched = True
                snap = det.snapshot or field
                t0 = _t.time()
                res = global_search(snap, pts)
                dt = _t.time() - t0
                tm = world_to_map(truth, spawn)
                err = math.hypot(res['pose'][0] - tm[0], res['pose'][1] - tm[1]) if res else float('nan')
                yerr = abs((res['pose'][2] - tm[2] + math.pi) % (2 * math.pi) - math.pi) if res else float('nan')
                searches.append((rid, round(err, 2), round(math.degrees(yerr), 1), round(res['score'], 3),
                                 round(res['second'], 3), round(res['margin'], 3), res['candidates'], round(dt, 2)))
                print(f'{rid}: search error {err:.2f} m, yaw {math.degrees(yerr):.1f} deg, score {res["score"]:.2f}, '
                      f'second {res["second"]:.2f}, {res["candidates"]} candidates, {dt:.2f} s', flush=True)
        print(f'{rid}: {sum(1 for r in rows if r[0] == rid)} scans scored', flush=True)
    with open(os.path.join(out_dir, 'scores.csv'), 'w') as f:
        f.write('run_id,world,t,phase,score,n_known,kidnap_flag\n')
        f.writelines(','.join(map(str, r)) + '\n' for r in rows)
    with open(os.path.join(out_dir, 'search.csv'), 'w') as f:
        f.write('run_id,pos_error_m,yaw_error_deg,score,second,margin,candidates,seconds\n')
        f.writelines(','.join(map(str, r)) + '\n' for r in searches)
    for ph in ('normal', 'after', 'late'):
        v = np.array([r[4] for r in rows if r[3] == ph and r[5] >= 30])
        if len(v):
            print(f'{ph}: n={len(v)} min {v.min():.2f} p1 {np.percentile(v, 1):.2f} p5 {np.percentile(v, 5):.2f} '
                  f'median {np.median(v):.2f} max {v.max():.2f}')
    for w in sorted({r[1] for r in rows}):
        flags = sum(r[6] for r in rows if r[1] == w and r[3] == 'normal')
        print(f'{w}: kidnap flags during normal driving: {flags}')


def lost(out_csv, *bag_dirs):
    """Lost-mode go/no-go: after each teleport, search every 2 s against the map from just before it.

    Per search: was a correct match possible at all (true cell known free in that map), was the answer
    confident (score >= 0.85, margin >= 0.15), was it correct (within 0.5 m and 15 deg of the truth).
    Then, per bag, the motion check: does any pair of confident answers >= 1 m / 45 deg of odometry
    apart agree with odometry, and is that pair correct?
    """
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rescue_mission'))
    from rescue_mission.relocalize import global_search, motion_consistent
    rows = []
    for bag in bag_dirs:
        rid = os.path.basename(bag.rstrip('/'))
        spawn = spawn_of(rid.split('_')[0])
        t_jump, _, maps = find_teleport(bag)
        snap_msg = [m for m in maps if m[0] < t_jump][-1][1]
        from rescue_mission.relocalize import Field
        snap = Field.from_msg(snap_msg)
        last, conf = -1e9, []
        for t, _field, pts, _slam, truth, odom in replay(bag, 0.5, with_odom=True):
            if t < t_jump + 3.0 or t - last < 2.0:
                continue
            last = t
            tm = world_to_map(truth, spawn)
            r, c, inside = snap.cells(np.array([[tm[0], tm[1]]]))
            possible = bool(inside[0] and snap.free[r[0], c[0]])
            res = global_search(snap, pts, min_known=40)      # as live: lost_min_known
            if res is None:
                rows.append((rid, round(t - t_jump, 1), int(possible), '', '', '', 0, '', ''))
                continue
            err = math.hypot(res['pose'][0] - tm[0], res['pose'][1] - tm[1])
            yerr = math.degrees(abs((res['pose'][2] - tm[2] + math.pi) % (2 * math.pi) - math.pi))
            confident = res['score'] >= 0.85 and res['margin'] >= 0.15
            correct = err <= 0.5 and yerr <= 15
            rows.append((rid, round(t - t_jump, 1), int(possible), round(res['score'], 3), round(res['margin'], 3),
                         res['n_known'], int(confident), round(err, 2), int(correct)))
            if confident:
                conf.append((t, res['pose'], odom, correct, res['n_known']))
        # motion check over confident answers, in time order: first accepted pair
        accepted = None
        for i in range(len(conf)):
            for j in range(i):
                ok, why = motion_consistent(conf[j][1], conf[j][2], conf[i][1], conf[i][2])
                if ok:
                    accepted = (conf[i][0] - t_jump, conf[j][3] and conf[i][3], why)
                    break
            if accepted:
                break
        mine = [r for r in rows if r[0] == rid]
        n_pos = sum(r[2] for r in mine)
        n_conf_imp = sum(1 for r in mine if not r[2] and r[6])
        n_conf_pos = sum(1 for r in mine if r[2] and r[6])
        n_conf_pos_ok = sum(1 for r in mine if r[2] and r[6] and r[8] == 1)
        first_ok = next((r[1] for r in mine if r[6] and r[8] == 1), None)
        print(f'{rid}: {len(mine)} searches; possible {n_pos}; confident when possible {n_conf_pos} '
              f'(correct {n_conf_pos_ok}, first correct at +{first_ok} s); confident when impossible '
              f'{n_conf_imp}; motion-checked accept: '
              f'{"none" if accepted is None else f"+{accepted[0]:.0f} s, correct={accepted[1]} ({accepted[2]})"}',
              flush=True)
    with open(out_csv, 'w') as f:
        f.write('run_id,t_after_teleport_s,possible,score,margin,n_known,confident,pos_error_m,correct\n')
        f.writelines(','.join(map(str, r)) + '\n' for r in rows)


if __name__ == '__main__':
    cmd = sys.argv[1]
    {'landing': landing, 'calibrate': calibrate, 'lost': lost}[cmd](*sys.argv[2:])
