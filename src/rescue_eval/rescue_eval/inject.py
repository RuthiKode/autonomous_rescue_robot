"""Scripted failures for the recovery tests (docs/PLAN.md, stage 7 gate).

  block     a wall appears 1.5 m ahead of the robot, across its heading (a blocked corridor)
  teleport  the robot is moved to another reachable spot far away (SLAM has to cope with the jump)

Pure helpers plus two thin wrappers around `gz service`.
"""
import math
import subprocess

import numpy as np


def block_pose_ahead(pose, distance=1.5):
    """(x, y, yaw) of a wall `distance` ahead of a robot pose (x, y, yaw); the wall's yaw equals
    the robot's, so a box that is thin along x and wide along y lies across the heading."""
    x, y, yaw = pose
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw


def block_sdf(x, y, yaw, width=1.6, thickness=0.3, height=1.0, name='injected_block'):
    size = f'{thickness} {width} {height}'
    geometry = f'<geometry><box><size>{size}</size></box></geometry>'
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<pose>{x:.3f} {y:.3f} {height / 2:.2f} 0 0 {yaw:.4f}</pose><link name="link">'
            f'<collision name="collision">{geometry}</collision>'
            f'<visual name="visual">{geometry}<material><diffuse>0.8 0.2 0.2 1</diffuse></material>'
            f'</visual></link></model></sdf>')


def pick_teleport_target(truth_cells_mask, resolution, origin, current_xy, rng, min_distance=5.0):
    """A random reachable cell centre at least min_distance from current_xy, as (x, y, yaw).

    truth_cells_mask is the boolean reachable-cell grid (row 0 = top, like the truth map);
    origin is the world position of the grid's lower-left corner.
    """
    h = truth_cells_mask.shape[0]
    rows, cols = np.nonzero(truth_cells_mask)
    x = origin[0] + (cols + 0.5) * resolution
    y = origin[1] + (h - 1 - rows + 0.5) * resolution
    far = np.hypot(x - current_xy[0], y - current_xy[1]) >= min_distance
    if not far.any():
        raise ValueError('no reachable cell is far enough from the robot')
    k = int(rng.choice(np.nonzero(far)[0]))
    return float(x[k]), float(y[k]), float(rng.uniform(-math.pi, math.pi))


def _gz(world, service, reqtype, req):
    r = subprocess.run(['gz', 'service', '-s', f'/world/{world}/{service}', '--reqtype', reqtype,
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True, check=False)
    return r.returncode == 0 and 'true' in r.stdout


def create_block(world, x, y, yaw):
    sdf = block_sdf(x, y, yaw).replace('"', '\\"')
    return _gz(world, 'create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')


def teleport(world, x, y, yaw, z=0.05):
    req = (f'name: "rescue_robot", position: {{x: {x:.3f}, y: {y:.3f}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
    return _gz(world, 'set_pose', 'gz.msgs.Pose', req)
