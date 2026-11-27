"""Unit tests for kidnap detection and global relocalization on synthetic maps."""
import math

import numpy as np

from rescue_mission.relocalize import Field, KidnapDetector, global_search, score

RES = 0.05


def l_room():
    """10 x 8 m room with an inner wall stub, so that no two poses look alike."""
    a = np.zeros((160, 200), dtype=np.int16)          # rows = y, cols = x
    a[0, :] = a[-1, :] = 100
    a[:, 0] = a[:, -1] = 100
    a[60:61, 0:90] = 100                              # stub from the left wall, y = 3 m, x 0..4.5 m
    a[100:160, 140:141] = 100                         # stub from the top wall, x = 7 m, y 5..8 m
    for (y, x, h, w) in [(20, 30, 6, 10), (110, 60, 10, 6), (40, 160, 8, 8), (130, 175, 6, 12)]:
        a[y:y + h, x:x + w] = 100                     # rubble boxes, as in the generated worlds
    return Field(a, RES, (0.0, 0.0))


def corridor():
    """A long featureless corridor: every position along it looks the same."""
    a = np.zeros((40, 400), dtype=np.int16)
    a[0, :] = a[-1, :] = 100
    return Field(a, RES, (0.0, 0.0))


def simulate_scan(field, pose, n=180, max_range=8.0):
    """Ray-cast endpoints (robot frame) against the occupied cells of field."""
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for r in np.arange(0.1, max_range, RES / 2):
            x = pose[0] + r * math.cos(pose[2] + a)
            y = pose[1] + r * math.sin(pose[2] + a)
            c, rr = int(x / RES), int(y / RES)
            if not (0 <= rr < field.shape[0] and 0 <= c < field.shape[1]):
                break
            if field.dist[rr, c] == 0.0:
                pts.append((r * math.cos(a), r * math.sin(a)))
                break
    return np.array(pts)


def test_scan_from_true_pose_scores_high_and_shifted_pose_low():
    f = l_room()
    true = (6.0, 2.0, 0.3)
    pts = simulate_scan(f, true)
    good, n = score(f, pts, true)
    bad, _ = score(f, pts, (2.0, 6.0, 0.3))
    assert n > 100
    assert good > 0.9
    assert bad < 0.5


def test_global_search_recovers_a_known_pose():
    f = l_room()
    true = (6.0, 2.0, 0.3)
    res = global_search(f, simulate_scan(f, true))
    assert res is not None
    assert math.hypot(res['pose'][0] - true[0], res['pose'][1] - true[1]) < 0.3
    assert abs((res['pose'][2] - true[2] + math.pi) % (2 * math.pi) - math.pi) < math.radians(10)
    assert res['margin'] > 0.1


def test_featureless_corridor_gives_no_margin():
    f = corridor()
    res = global_search(f, simulate_scan(f, (10.0, 1.0, 0.0), max_range=3.0))
    assert res is not None
    assert res['margin'] < 0.1           # many equally good places: the caller must not commit


def test_detector_needs_consecutive_low_scores_and_keeps_good_snapshot():
    f = l_room()
    true = (6.0, 2.0, 0.3)
    pts = simulate_scan(f, true)
    det = KidnapDetector(low_score=0.35, checks=3, min_known=30, min_known_area_m2=5.0)
    assert det.update(f, pts, true)[0] is False
    assert det.snapshot is f
    wrong = (2.0, 6.0, 0.3)
    assert det.update(f, pts, wrong)[0] is False
    assert det.update(f, pts, wrong)[0] is False
    assert det.update(f, pts, wrong)[0] is True
    det.reset()
    assert det.update(f, pts, true)[0] is False


def test_detector_ignores_small_maps_and_scans_into_unknown():
    a = np.full((160, 200), -1, dtype=np.int16)
    a[70:90, 90:110] = 0                              # a small known patch only
    f = Field(a, RES, (0.0, 0.0))
    det = KidnapDetector(min_known_area_m2=20.0)
    kid, s, n = det.update(f, np.array([[1.0, 0.0], [0.0, 1.0]]), (5.0, 4.0, 0.0))
    assert kid is False and n == 0
