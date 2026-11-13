"""Seeded generator for the disaster worlds.

Writes, for every world in the config, an SDF file and a ground-truth YAML file
with the survivor poses:

  ros2 run rescue_simulation generate_worlds --output-dir src/rescue_simulation/worlds

The same seed and config always produce byte-identical files.
"""
import argparse
import math
import os
import random
import sys

import yaml

WALL_RGB = (0.78, 0.78, 0.74)
CONCRETE_RGB = [(0.55, 0.55, 0.53), (0.45, 0.44, 0.42), (0.62, 0.60, 0.56), (0.40, 0.36, 0.32)]
FURNITURE_RGB = [(0.50, 0.33, 0.18), (0.35, 0.35, 0.38), (0.62, 0.52, 0.36), (0.25, 0.30, 0.38)]
SHIRT_RGB = [(0.80, 0.10, 0.10), (0.10, 0.25, 0.75), (0.95, 0.75, 0.10), (0.10, 0.55, 0.25),
             (0.90, 0.90, 0.90)]
TROUSER_RGB = [(0.12, 0.14, 0.30), (0.20, 0.20, 0.20), (0.35, 0.28, 0.18)]
SKIN_RGB = [(0.95, 0.78, 0.65), (0.78, 0.57, 0.42), (0.48, 0.33, 0.24)]

POSTURES = ('standing', 'sitting', 'lying')
# Radius kept clear around a survivor, and height of the body centroid.
SURVIVOR_RADIUS = {'standing': 0.6, 'sitting': 0.9, 'lying': 1.0}
SURVIVOR_CENTROID_Z = {'standing': 0.85, 'sitting': 0.40, 'lying': 0.11}


def _fmt(*values):
    return ' '.join(f'{v:.4f}' for v in values)


def _round(v):
    return round(float(v), 4)


class WorldBuilder:
    """Collects static geometry and survivors for one world, then emits SDF and YAML."""

    def __init__(self, name, cfg, common, seed):
        self.name = name
        self.cfg = cfg
        self.common = common
        self.seed = seed
        self.rng = random.Random(f'{seed}:{name}')
        self.shapes = []
        self.survivors = []
        self.keepout = []
        self.spawn = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.reserve(0.0, 0.0, common['spawn_clear_radius'])

    # ---------------- free-space bookkeeping ----------------
    def reserve(self, x, y, r):
        self.keepout.append((x, y, r))

    def is_free(self, x, y, r):
        return all(math.hypot(x - cx, y - cy) >= r + cr for cx, cy, cr in self.keepout)

    def sample(self, rect, r, tries=300):
        """Return a free point at least r from the rect border and all keep-outs, or None."""
        xmin, ymin, xmax, ymax = rect
        if xmax - xmin < 2 * r or ymax - ymin < 2 * r:
            return None
        for _ in range(tries):
            x = self.rng.uniform(xmin + r, xmax - r)
            y = self.rng.uniform(ymin + r, ymax - r)
            if self.is_free(x, y, r):
                return x, y
        return None

    # ---------------- geometry ----------------
    def add_box(self, size, pose, rgb):
        self.shapes.append({'geom': 'box', 'size': tuple(size), 'pose': tuple(pose), 'rgb': rgb})

    def add_cylinder(self, radius, length, pose, rgb):
        self.shapes.append({'geom': 'cylinder', 'size': (radius, length),
                            'pose': tuple(pose), 'rgb': rgb})

    def add_wall(self, x1, y1, x2, y2, height=None, thickness=None, rgb=WALL_RGB):
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 1e-6:
            return
        height = self.common['wall_height'] if height is None else height
        thickness = self.common['wall_thickness'] if thickness is None else thickness
        yaw = math.atan2(y2 - y1, x2 - x1)
        self.add_box((length, thickness, height),
                     ((x1 + x2) / 2, (y1 + y2) / 2, height / 2, 0.0, 0.0, yaw), rgb)

    def outer_walls(self, hx, hy):
        self.add_wall(-hx, -hy, hx, -hy)
        self.add_wall(-hx, hy, hx, hy)
        self.add_wall(-hx, -hy, -hx, hy)
        self.add_wall(hx, -hy, hx, hy)

    def interior(self, hx, hy, margin=0.0):
        t = self.common['wall_thickness'] / 2 + margin
        return (-hx + t, -hy + t, hx - t, hy - t)

    # ---------------- survivors ----------------
    def survivor_plan(self):
        """Return [(posture, occluded)], always mixing standing, sitting and lying."""
        n = self.rng.randint(self.common['survivors']['min'], self.common['survivors']['max'])
        postures = ['standing', 'sitting', 'lying', 'lying', 'standing', 'sitting'][:n]
        self.rng.shuffle(postures)
        n_occluded = max(1, round(n * self.common['occluded_fraction']))
        occluded = set(self.rng.sample(range(n), n_occluded))
        return [(postures[i], i in occluded) for i in range(n)]

    def add_survivor(self, x, y, yaw, posture, occluded):
        self.survivors.append({
            'id': len(self.survivors) + 1,
            'posture': posture,
            'occluded': occluded,
            'x': x, 'y': y, 'z': SURVIVOR_CENTROID_Z[posture], 'yaw': yaw,
            'shirt': self.rng.choice(SHIRT_RGB),
            'trousers': self.rng.choice(TROUSER_RGB),
            'skin': self.rng.choice(SKIN_RGB),
        })
        if occluded:
            self._add_occluder(x, y, yaw, posture)
        self.reserve(x, y, SURVIVOR_RADIUS[posture])

    def _add_occluder(self, x, y, yaw, posture):
        """Place debris that hides part of the body (legs / lower half)."""
        if posture == 'lying':
            local, size = (-0.45, 0.0, 0.15), (0.75, 0.65, 0.30)
        elif posture == 'sitting':
            local, size = (0.45, 0.0, 0.25), (0.14, 0.90, 0.50)
        else:
            local, size = (0.30, 0.0, 0.45), (0.14, 0.90, 0.90)
        wx = x + local[0] * math.cos(yaw) - local[1] * math.sin(yaw)
        wy = y + local[0] * math.sin(yaw) + local[1] * math.cos(yaw)
        skew = self.rng.uniform(-0.15, 0.15)
        self.add_box(size, (wx, wy, local[2], 0.0, 0.0, yaw + skew), self.rng.choice(CONCRETE_RGB))

    # ---------------- output ----------------
    def to_sdf(self):
        c = self.common
        shadows = 'true' if c['shadows'] else 'false'
        out = [
            '<?xml version="1.0"?>',
            f'<!-- Generated by rescue_simulation generate_worlds (seed {self.seed}). Do not edit. -->',
            '<sdf version="1.9">',
            f'  <world name="{self.name}">',
            '    <physics name="default" type="ignored">',
            f'      <max_step_size>{c["physics_step_s"]}</max_step_size>',
            f'      <real_time_factor>{c["real_time_factor"]}</real_time_factor>',
            '    </physics>',
            '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
            '    <plugin filename="gz-sim-user-commands-system"'
            ' name="gz::sim::systems::UserCommands"/>',
            '    <plugin filename="gz-sim-scene-broadcaster-system"'
            ' name="gz::sim::systems::SceneBroadcaster"/>',
            '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
            '      <render_engine>ogre2</render_engine>',
            '    </plugin>',
            '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
            '    <scene>',
            '      <ambient>0.55 0.55 0.55 1</ambient>',
            '      <background>0.70 0.72 0.75 1</background>',
            f'      <shadows>{shadows}</shadows>',
            '    </scene>',
            '    <light type="directional" name="sun">',
            f'      <cast_shadows>{shadows}</cast_shadows>',
            '      <pose>0 0 10 0 0 0</pose>',
            '      <diffuse>0.8 0.8 0.8 1</diffuse>',
            '      <specular>0.2 0.2 0.2 1</specular>',
            '      <direction>-0.4 0.2 -0.9</direction>',
            '    </light>',
            '    <model name="ground_plane">',
            '      <static>true</static>',
            '      <link name="link">',
            '        <collision name="collision">',
            '          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            '          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>',
            '          <material><ambient>0.35 0.35 0.35 1</ambient>'
            '<diffuse>0.35 0.35 0.35 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
            '    <model name="structure">',
            '      <static>true</static>',
            '      <link name="link">',
        ]
        for i, shape in enumerate(self.shapes):
            out.extend(_shape_sdf(f's{i}', shape, indent='        '))
        out.extend(['      </link>', '    </model>'])
        for s in self.survivors:
            out.extend([
                f'    <model name="survivor_{s["id"]}">',
                '      <static>true</static>',
                f'      <pose>{_fmt(s["x"], s["y"], 0.0, 0.0, 0.0, s["yaw"])}</pose>',
                '      <link name="link">',
            ])
            for i, shape in enumerate(_survivor_shapes(s)):
                out.extend(_shape_sdf(f'p{i}', shape, indent='        '))
            out.extend(['      </link>', '    </model>'])
        out.extend(['  </world>', '</sdf>', ''])
        return '\n'.join(out)

    def to_truth(self):
        truth = {
            'world': self.name,
            'seed': self.seed,
            'frame_id': 'map',
            'bounds': {'size_x': _round(self.cfg['size_x']), 'size_y': _round(self.cfg['size_y'])},
            'robot_spawn': {k: _round(v) for k, v in self.spawn.items()},
            'survivors': [{
                'id': s['id'],
                'posture': s['posture'],
                'occluded': bool(s['occluded']),
                'position': {'x': _round(s['x']), 'y': _round(s['y']), 'z': _round(s['z'])},
                'yaw': _round(s['yaw']),
            } for s in self.survivors],
        }
        if 'gap_width' in self.cfg:
            truth['gap_width'] = _round(self.cfg['gap_width'])
        return truth


def _shape_sdf(name, shape, indent):
    if shape['geom'] == 'box':
        geom = f'<box><size>{_fmt(*shape["size"])}</size></box>'
    elif shape['geom'] == 'cylinder':
        geom = (f'<cylinder><radius>{_fmt(shape["size"][0])}</radius>'
                f'<length>{_fmt(shape["size"][1])}</length></cylinder>')
    else:
        geom = f'<sphere><radius>{_fmt(shape["size"][0])}</radius></sphere>'
    rgb = _fmt(*shape['rgb'])
    pose = _fmt(*shape['pose'])
    return [
        f'{indent}<collision name="{name}_c"><pose>{pose}</pose>'
        f'<geometry>{geom}</geometry></collision>',
        f'{indent}<visual name="{name}_v"><pose>{pose}</pose><geometry>{geom}</geometry>'
        f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
        f'<specular>0.1 0.1 0.1 1</specular></material></visual>',
    ]


def _survivor_shapes(s):
    """Low-poly person built from primitives, in the model frame (origin on the ground)."""
    half_pi = math.pi / 2

    def box(size, pose, rgb):
        return {'geom': 'box', 'size': size, 'pose': pose, 'rgb': rgb}

    def cyl(radius, length, pose, rgb):
        return {'geom': 'cylinder', 'size': (radius, length), 'pose': pose, 'rgb': rgb}

    def sphere(radius, pose, rgb):
        return {'geom': 'sphere', 'size': (radius,), 'pose': pose, 'rgb': rgb}

    shirt, trousers, skin = s['shirt'], s['trousers'], s['skin']
    if s['posture'] == 'standing':
        return [
            cyl(0.07, 0.80, (0.0, 0.09, 0.40, 0, 0, 0), trousers),
            cyl(0.07, 0.80, (0.0, -0.09, 0.40, 0, 0, 0), trousers),
            box((0.22, 0.38, 0.60), (0.0, 0.0, 1.10, 0, 0, 0), shirt),
            cyl(0.05, 0.58, (0.0, 0.24, 1.10, 0, 0, 0), shirt),
            cyl(0.05, 0.58, (0.0, -0.24, 1.10, 0, 0, 0), shirt),
            sphere(0.11, (0.0, 0.0, 1.53, 0, 0, 0), skin),
        ]
    if s['posture'] == 'sitting':  # on the floor, legs along +x
        return [
            cyl(0.07, 0.80, (0.40, 0.09, 0.08, 0, half_pi, 0), trousers),
            cyl(0.07, 0.80, (0.40, -0.09, 0.08, 0, half_pi, 0), trousers),
            box((0.22, 0.38, 0.60), (-0.05, 0.0, 0.40, 0, 0, 0), shirt),
            cyl(0.05, 0.50, (-0.05, 0.24, 0.40, 0, 0, 0), shirt),
            cyl(0.05, 0.50, (-0.05, -0.24, 0.40, 0, 0, 0), shirt),
            sphere(0.11, (-0.05, 0.0, 0.83, 0, 0, 0), skin),
        ]
    # lying on the back, head towards +x, body centred on the origin
    return [
        cyl(0.07, 0.80, (-0.45, 0.09, 0.09, 0, half_pi, 0), trousers),
        cyl(0.07, 0.80, (-0.45, -0.09, 0.09, 0, half_pi, 0), trousers),
        box((0.60, 0.38, 0.22), (0.25, 0.0, 0.11, 0, 0, 0), shirt),
        cyl(0.05, 0.58, (0.25, 0.24, 0.07, 0, half_pi, 0), shirt),
        cyl(0.05, 0.58, (0.25, -0.24, 0.07, 0, half_pi, 0), shirt),
        sphere(0.11, (0.68, 0.0, 0.11, 0, 0, 0), skin),
    ]


# ============================ layouts ============================

def _place_survivor(b, rect, posture, occluded, extra_margin=0.2):
    pos = b.sample(rect, SURVIVOR_RADIUS[posture] + extra_margin)
    if pos is None:
        raise RuntimeError(f'{b.name}: no free space left for a {posture} survivor')
    b.add_survivor(pos[0], pos[1], b.rng.uniform(-math.pi, math.pi), posture, occluded)


def _place_box(b, rect, size, margin=0.3, yaw=None, rgb=None):
    radius = math.hypot(size[0], size[1]) / 2 + margin
    pos = b.sample(rect, radius)
    if pos is None:
        return False
    if yaw is None:
        yaw = b.rng.choice([0.0, math.pi / 2]) + b.rng.uniform(-0.2, 0.2)
    b.add_box(size, (pos[0], pos[1], size[2] / 2, 0.0, 0.0, yaw),
              rgb or b.rng.choice(FURNITURE_RGB))
    b.reserve(pos[0], pos[1], radius)
    return True


def build_rooms(b):
    """A central corridor along x with a row of rooms on each side, one door per room."""
    c = b.cfg
    hx, hy = c['size_x'] / 2, c['size_y'] / 2
    cw, n, dw = c['corridor_width'] / 2, c['rooms_per_side'], c['door_width']
    t = b.common['wall_thickness'] / 2
    room_w = 2 * hx / n
    if room_w < dw + 1.2:
        raise ValueError('rooms: door_width is too large for the room width')
    b.outer_walls(hx, hy)
    rooms = []
    for side in (1, -1):
        start = -hx
        for i in range(n):
            x0, x1 = -hx + i * room_w, -hx + (i + 1) * room_w
            door_x = b.rng.uniform(x0 + dw / 2 + 0.5, x1 - dw / 2 - 0.5)
            b.add_wall(start, side * cw, door_x - dw / 2, side * cw)
            start = door_x + dw / 2
            b.reserve(door_x, side * (cw + 0.6), 0.8)  # keep the doorway clear
            if i > 0:
                b.add_wall(x0, side * cw, x0, side * hy)
            ya, yb = sorted((side * (cw + t), side * (hy - t)))
            rooms.append((x0 + t, ya, x1 - t, yb))
        b.add_wall(start, side * cw, hx, side * cw)

    plan = b.survivor_plan()
    for (posture, occluded), room in zip(plan, b.rng.sample(rooms, len(plan))):
        _place_survivor(b, room, posture, occluded)
    for room in rooms:
        for _ in range(c['furniture_per_room']):
            size = (b.rng.uniform(0.5, 1.3), b.rng.uniform(0.4, 0.8), b.rng.uniform(0.4, 1.1))
            _place_box(b, room, size)


def _gap_positions(b, lo, hi, n, gap_width, avoid=()):
    """Pick n gap centres in [lo, hi], one per equal slot, away from the ends and `avoid`."""
    end_margin = 1.2
    slot = (hi - lo - 2 * end_margin) / n
    pad = gap_width / 2 + 0.6
    if slot <= 2 * pad:
        raise ValueError(f'{b.name}: gap_width {gap_width} does not fit {n} gaps in a barrier')
    centres = []
    for i in range(n):
        s0 = lo + end_margin + i * slot
        for _ in range(200):
            centre = b.rng.uniform(s0 + pad, s0 + slot - pad)
            if all(abs(centre - a) >= gap_width / 2 + 1.0 for a in avoid):
                break
        else:
            raise ValueError(f'{b.name}: could not place a gap clear of the barrier crossing')
        centres.append(centre)
    return centres


def _rubble_barrier(b, axis, fixed, lo, hi, gaps, gap_width):
    """Chain of concrete blocks along one axis, interrupted by gaps of exactly gap_width."""
    edges = [lo]
    for g in sorted(gaps):
        edges.extend([g - gap_width / 2, g + gap_width / 2])
    edges.append(hi)
    for a, e in zip(edges[0::2], edges[1::2]):
        pos = a
        while e - pos > 1e-6:
            length = b.rng.uniform(0.8, 1.6)
            if e - pos - length < 0.5:
                length = e - pos
            thickness = b.rng.uniform(0.3, 0.5)
            height = b.rng.uniform(0.5, 1.6)
            mid = pos + length / 2
            if axis == 'y':
                b.add_box((thickness, length, height), (fixed, mid, height / 2, 0, 0, 0),
                          b.rng.choice(CONCRETE_RGB))
            else:
                b.add_box((length, thickness, height), (mid, fixed, height / 2, 0, 0, 0),
                          b.rng.choice(CONCRETE_RGB))
            pos += length
    for g in gaps:
        if axis == 'y':
            b.reserve(fixed, g, 1.0)
        else:
            b.reserve(g, fixed, 1.0)


def build_collapsed(b):
    """A collapsed floor: rubble barriers with narrow gaps split it into three zones."""
    c = b.cfg
    hx, hy = c['size_x'] / 2, c['size_y'] / 2
    gw, bx, by = c['gap_width'], c['barrier_x'], c['barrier_y']
    b.outer_walls(hx, hy)
    gaps_a = _gap_positions(b, -hy, hy, c['gaps_per_barrier'], gw, avoid=(by,))
    _rubble_barrier(b, 'y', bx, -hy, hy, gaps_a, gw)
    gaps_b = _gap_positions(b, -hx, bx, c['gaps_per_barrier'], gw)
    _rubble_barrier(b, 'x', by, -hx, bx - 0.25, gaps_b, gw)

    m = 0.3  # stay clear of walls and barriers
    zones = [(-hx + m, -hy + m, bx - m, by - m),
             (-hx + m, by + m, bx - m, hy - m),
             (bx + m, -hy + m, hx - m, hy - m)]

    plan = b.survivor_plan()
    order = list(range(len(zones)))
    b.rng.shuffle(order)
    for i, (posture, occluded) in enumerate(plan):
        _place_survivor(b, zones[order[i % len(zones)]], posture, occluded)

    for _ in range(c['fallen_slabs']):
        sx, sy = b.rng.uniform(1.2, 2.0), b.rng.uniform(0.8, 1.2)
        pitch = b.rng.uniform(0.25, 0.6)
        radius = sx / 2 + 0.4
        pos = b.sample(b.rng.choice(zones), radius)
        if pos is None:
            continue
        z = (sx / 2) * math.sin(pitch) + 0.06
        b.add_box((sx, sy, 0.12), (pos[0], pos[1], z, 0.0, pitch, b.rng.uniform(-math.pi, math.pi)),
                  b.rng.choice(CONCRETE_RGB))
        b.reserve(pos[0], pos[1], radius)

    for _ in range(c['debris_clusters']):
        centre = b.sample(b.rng.choice(zones), 1.3)
        if centre is None:
            continue
        for _ in range(b.rng.randint(c['debris_pieces']['min'], c['debris_pieces']['max'])):
            dx = max(-0.9, min(0.9, b.rng.gauss(0.0, 0.45)))
            dy = max(-0.9, min(0.9, b.rng.gauss(0.0, 0.45)))
            size = (b.rng.uniform(0.15, 0.7), b.rng.uniform(0.15, 0.6), b.rng.uniform(0.08, 0.45))
            rpy = (b.rng.uniform(-0.3, 0.3), b.rng.uniform(-0.3, 0.3),
                   b.rng.uniform(-math.pi, math.pi))
            b.add_box(size, (centre[0] + dx, centre[1] + dy, size[2] / 2 + 0.03) + rpy,
                      b.rng.choice(CONCRETE_RGB))
        b.reserve(centre[0], centre[1], 1.3)


def build_open_plan(b):
    """A large open floor with a pillar grid and scattered clutter."""
    c = b.cfg
    hx, hy = c['size_x'] / 2, c['size_y'] / 2
    b.outer_walls(hx, hy)
    ps = c['pillar_size']
    nx = int(round(2 * hx / c['pillar_spacing_x']))
    ny = int(round(2 * hy / c['pillar_spacing_y']))
    for i in range(nx):
        for j in range(ny):
            px = -hx + (i + 0.5) * c['pillar_spacing_x']
            py = -hy + (j + 0.5) * c['pillar_spacing_y']
            if not b.is_free(px, py, ps):
                continue
            height = b.common['wall_height']
            b.add_box((ps, ps, height), (px, py, height / 2, 0, 0, 0), WALL_RGB)
            b.reserve(px, py, ps / 2 + 0.2)

    area = b.interior(hx, hy, margin=0.2)
    for posture, occluded in b.survivor_plan():
        spread = max(0.2, c['survivor_spacing'] - 2 * SURVIVOR_RADIUS[posture])
        for _ in range(300):
            pos = b.sample(area, SURVIVOR_RADIUS[posture] + spread)
            if pos is not None and math.hypot(*pos) >= c['min_survivor_distance']:
                break
        else:
            raise RuntimeError('open_plan: no free space left for a survivor')
        b.add_survivor(pos[0], pos[1], b.rng.uniform(-math.pi, math.pi), posture, occluded)

    presets = [
        ('box', (1.4, 0.7, 0.75)),    # desk
        ('box', (0.5, 0.9, 1.30)),    # cabinet
        ('box', (2.0, 0.08, 1.40)),   # partition
        ('box', (0.6, 0.6, 0.60)),    # crate
        ('cylinder', (0.3, 0.9)),     # barrel
    ]
    for _ in range(c['clutter_items']):
        kind, size = b.rng.choice(presets)
        if kind == 'box':
            _place_box(b, area, size, yaw=b.rng.uniform(-math.pi, math.pi))
        else:
            radius = size[0] + 0.3
            pos = b.sample(area, radius)
            if pos is None:
                continue
            b.add_cylinder(size[0], size[1], (pos[0], pos[1], size[1] / 2, 0, 0, 0),
                           b.rng.choice(FURNITURE_RGB))
            b.reserve(pos[0], pos[1], radius)


LAYOUTS = {'rooms': build_rooms, 'collapsed': build_collapsed, 'open_plan': build_open_plan}


# ============================ entry points ============================

def generate(name, world_cfg, common, seed):
    """Build one world. Returns (sdf_text, ground_truth_dict)."""
    builder = WorldBuilder(name, world_cfg, common, seed)
    LAYOUTS[world_cfg['layout']](builder)
    return builder.to_sdf(), builder.to_truth()


def default_config_path():
    here = os.path.dirname(os.path.abspath(__file__))
    source = os.path.join(here, '..', 'config', 'world_generator.yaml')
    if os.path.isfile(source):
        return source
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rescue_simulation'),
                        'config', 'world_generator.yaml')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--output-dir', required=True, help='directory for .sdf and .yaml files')
    parser.add_argument('--config', default=None, help='world_generator.yaml (default: packaged)')
    parser.add_argument('--seed', type=int, default=None, help='override the seed in the config')
    parser.add_argument('--gap-width', type=float, default=None,
                        help='override gap_width of every world that has one')
    parser.add_argument('--world', action='append', default=None,
                        help='generate only this world (repeatable)')
    args = parser.parse_args(argv)

    with open(args.config or default_config_path()) as f:
        cfg = yaml.safe_load(f)
    seed = cfg['seed'] if args.seed is None else args.seed
    names = args.world or list(cfg['worlds'])
    os.makedirs(args.output_dir, exist_ok=True)
    for name in names:
        if name not in cfg['worlds']:
            sys.exit(f"Unknown world '{name}'. Available: {', '.join(cfg['worlds'])}")
        world_cfg = dict(cfg['worlds'][name])
        if args.gap_width is not None and 'gap_width' in world_cfg:
            world_cfg['gap_width'] = args.gap_width
        sdf, truth = generate(name, world_cfg, cfg['common'], seed)
        with open(os.path.join(args.output_dir, name + '.sdf'), 'w') as f:
            f.write(sdf)
        with open(os.path.join(args.output_dir, name + '_survivors.yaml'), 'w') as f:
            f.write('# Generated ground truth. Poses are in the map frame'
                    ' (robot spawn = origin).\n')
            yaml.safe_dump(truth, f, sort_keys=False)
        postures = ', '.join(s['posture'] + ('*' if s['occluded'] else '')
                             for s in truth['survivors'])
        print(f"{name}: seed {seed}, {len(truth['survivors'])} survivors ({postures})"
              f" [* = occluded]")


if __name__ == '__main__':
    main()
