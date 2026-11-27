"""Kidnap detection and global relocalization against the SLAM occupancy grid (pure numpy/scipy).

Detection: project the latest scan from the SLAM pose into the map and look only at the endpoints
that land on *known* cells. A localized robot puts most of them next to a wall; a kidnapped robot,
whose SLAM pose is wrong, puts them in open space. score = share of known-cell endpoints within
hit_dist_m of an occupied cell (soft: each endpoint counts exp(-d^2 / 2 sigma^2), sigma = hit_dist_m / 2).

Relocalization: score the scan at every known-free cell of a grid (coarse step) and a set of yaws,
refine the best few, and return the best pose with its score and its margin over the best pose
that is clearly elsewhere (a symmetric map gives a small margin, and the caller should not commit).
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

OCCUPIED = 65   # OccupancyGrid value at or above which a cell is a wall (nav2 map_saver default)


class Field:
    """Distance from every cell to the nearest wall, in metres, plus which cells are known."""

    def __init__(self, data, resolution, origin):
        a = np.asarray(data, dtype=np.int16)
        self.res, self.origin = float(resolution), (float(origin[0]), float(origin[1]))
        self.known = a >= 0
        self.free = (a >= 0) & (a < OCCUPIED)
        occ = a >= OCCUPIED
        if occ.any():
            self.dist = distance_transform_edt(~occ) * self.res
        else:
            self.dist = np.full(a.shape, np.inf)
        self.shape = a.shape

    @classmethod
    def from_msg(cls, msg):
        a = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        return cls(a, msg.info.resolution, (o.x, o.y))

    def known_area_m2(self):
        return float(self.known.sum()) * self.res * self.res

    def cells(self, pts):
        """Map-frame points (N, 2) -> (row, col, inside mask)."""
        c = np.floor((pts[:, 0] - self.origin[0]) / self.res).astype(int)
        r = np.floor((pts[:, 1] - self.origin[1]) / self.res).astype(int)
        inside = (r >= 0) & (r < self.shape[0]) & (c >= 0) & (c < self.shape[1])
        return r, c, inside


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, sensor_pose=(0.0, 0.0, 0.0),
                max_points=90):
    """Valid scan endpoints in the robot (base) frame, subsampled to at most max_points."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max * 0.98)
    r, a = r[ok], a[ok]
    if len(r) > max_points:
        idx = np.linspace(0, len(r) - 1, max_points).astype(int)
        r, a = r[idx], a[idx]
    sx, sy, sth = sensor_pose
    x = sx + r * np.cos(a + sth)
    y = sy + r * np.sin(a + sth)
    return np.stack([x, y], axis=1)


def transform(pts, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def _soft(d, hit_dist_m):
    sigma = hit_dist_m / 2.0
    return np.exp(-0.5 * (d / sigma) ** 2)


def score(field, pts_base, pose, hit_dist_m=0.15):
    """(share of known-cell endpoints within hit_dist_m of a wall, number of known-cell endpoints)."""
    r, c, inside = field.cells(transform(pts_base, pose))
    r, c = r[inside], c[inside]
    known = field.known[r, c]
    n = int(known.sum())
    if n == 0:
        return 0.0, 0
    return float(np.mean(_soft(field.dist[r[known], c[known]], hit_dist_m))), n


def _batch_scores(field, pts_base, poses, hit_dist_m, min_known):
    """Scores for many poses at once: (P,) array, with -1 where fewer than min_known endpoints are known."""
    xs, ys, th = poses[:, 0:1], poses[:, 1:2], poses[:, 2:3]
    c, s = np.cos(th), np.sin(th)
    px = xs + c * pts_base[None, :, 0] - s * pts_base[None, :, 1]
    py = ys + s * pts_base[None, :, 0] + c * pts_base[None, :, 1]
    cc = np.floor((px - field.origin[0]) / field.res).astype(int)
    rr = np.floor((py - field.origin[1]) / field.res).astype(int)
    inside = (rr >= 0) & (rr < field.shape[0]) & (cc >= 0) & (cc < field.shape[1])
    rr, cc = np.where(inside, rr, 0), np.where(inside, cc, 0)
    known = inside & field.known[rr, cc]
    hit = np.where(known, _soft(field.dist[rr, cc], hit_dist_m), 0.0)
    n = known.sum(axis=1)
    # Endpoints on known cells score by their distance to a wall. Endpoints outside the grid count as
    # misses: the grid covers everything mapped so far, so a wall the scan sees must be inside it at the
    # true pose (unknown cells inside the grid stay neutral: the map may simply not have them yet).
    denom = n + (~inside).sum(axis=1)
    out = np.where(n >= min_known, hit.sum(axis=1) / np.maximum(denom, 1), -1.0)
    # a pose whose own cell is not free is impossible
    r0, c0, in0 = field.cells(poses[:, :2])
    free0 = np.zeros(len(poses), bool)
    free0[in0] = field.free[r0[in0], c0[in0]]
    return np.where(free0, out, -1.0)


def global_search(field, pts_base, coarse_step_m=0.25, n_yaw=24, fine_step_m=0.05, fine_span_m=0.25,
                  fine_yaw_rad=math.radians(8), fine_yaw_steps=9, top_k=8, hit_dist_m=0.15, min_known=20,
                  clearance_m=0.2, distinct_m=1.0, chunk=4000):
    """Best pose for the scan anywhere in the map's known free space.

    Returns dict(pose, score, second, margin, candidates) or None when nothing could be scored.
    second is the best score at least distinct_m away from the best pose.
    """
    stride = max(1, int(round(coarse_step_m / field.res)))
    ok = field.free & (field.dist >= clearance_m)
    rows, cols = np.nonzero(ok[::stride, ::stride])
    if len(rows) == 0:
        return None
    xs = field.origin[0] + (cols * stride + 0.5) * field.res
    ys = field.origin[1] + (rows * stride + 0.5) * field.res
    yaws = np.linspace(-math.pi, math.pi, n_yaw, endpoint=False)
    poses = np.array([(x, y, t) for x, y in zip(xs, ys) for t in yaws])
    scores = np.concatenate([_batch_scores(field, pts_base, poses[i:i + chunk], hit_dist_m, min_known)
                             for i in range(0, len(poses), chunk)])
    if scores.max() < 0:
        return None
    # refine the best coarse poses, one per location
    order = np.argsort(-scores)
    seeds = []
    for i in order:
        if scores[i] < 0 or len(seeds) >= top_k:
            break
        if all(math.hypot(poses[i, 0] - p[0], poses[i, 1] - p[1]) > coarse_step_m * 1.5 for p in seeds):
            seeds.append(poses[i])
    d = np.arange(-fine_span_m, fine_span_m + 1e-9, fine_step_m)
    dt = np.linspace(-fine_yaw_rad, fine_yaw_rad, fine_yaw_steps)
    refined = []
    for sd in seeds:
        grid = np.array([(sd[0] + a, sd[1] + b, sd[2] + t) for a in d for b in d for t in dt])
        sc = _batch_scores(field, pts_base, grid, hit_dist_m, min_known)
        j = int(np.argmax(sc))
        refined.append((float(sc[j]), grid[j]))
    refined.sort(key=lambda x: -x[0])
    best_score, best = refined[0]
    second = max([s for s, p in refined[1:] if math.hypot(p[0] - best[0], p[1] - best[1]) >= distinct_m],
                 default=0.0)
    yaw = (best[2] + math.pi) % (2 * math.pi) - math.pi
    _, n_known = score(field, pts_base, best, hit_dist_m)
    return {'pose': (float(best[0]), float(best[1]), float(yaw)), 'score': best_score,
            'second': float(second), 'margin': best_score - float(second), 'candidates': len(poses),
            'n_known': n_known}


def relative(a, b):
    """Pose b expressed in the frame of pose a: (dx, dy, dyaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi


def motion_consistent(match_a, odom_a, match_b, odom_b, min_motion_m=1.0, min_turn_rad=math.radians(45),
                      tol_m=0.3, tol_rad=math.radians(10)):
    """Two map matches taken at odometry poses odom_a, odom_b agree with each other: the robot moved
    enough between them (an alias seen twice from the same spot proves nothing), and the change between
    the two matches equals the change in odometry within tol. Returns (ok, why)."""
    od = relative(odom_a, odom_b)
    if math.hypot(od[0], od[1]) < min_motion_m and abs(od[2]) < min_turn_rad:
        return False, 'not enough motion between the two matches'
    md = relative(match_a, match_b)
    err = math.hypot(md[0] - od[0], md[1] - od[1])
    yerr = abs((md[2] - od[2] + math.pi) % (2 * math.pi) - math.pi)
    if err > tol_m or yerr > tol_rad:
        return False, f'matches disagree with odometry by {err:.2f} m, {math.degrees(yerr):.0f} deg'
    return True, f'matches agree with odometry within {err:.2f} m, {math.degrees(yerr):.0f} deg'


class KidnapDetector:
    """Counts low scan-to-map scores in a row; trusts a map snapshot only while scores are good."""

    def __init__(self, low_score=0.35, checks=3, min_known=30, min_known_area_m2=20.0, good_score=0.5):
        self.low_score, self.checks, self.min_known = low_score, checks, min_known
        self.min_area, self.good_score = min_known_area_m2, good_score
        self.low_count = 0
        self.snapshot = None          # Field from the last check with a good score

    def update(self, field, pts_base, slam_pose, hit_dist_m=0.15):
        """Returns (kidnapped?, score, n_known). Call once per check with the current map."""
        if field.known_area_m2() < self.min_area:
            return False, float('nan'), 0
        s, n = score(field, pts_base, slam_pose, hit_dist_m)
        if n < self.min_known:
            return False, s, n
        if s >= self.good_score:
            self.snapshot = field
        if s < self.low_score:
            self.low_count += 1
        else:
            self.low_count = 0
        return self.low_count >= self.checks, s, n

    def reset(self):
        self.low_count = 0
