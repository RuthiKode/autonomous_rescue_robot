"""Unit tests for the TF audit, the map score, the trajectory error and drive files."""
import math

import numpy as np
from PIL import Image
import pytest
import yaml

from rescue_eval import map_score, trajectory
from rescue_eval.cmd_vel_io import load_drive, save_drive
from rescue_eval.tf_audit import find_violations


# ---------------- TF audit ----------------
def test_single_publishers_pass():
    obs = []
    for seq in range(1, 20):
        obs.append(('/tf', 'odom', 'base_link', seq))          # EKF
        obs.append(('/tf', 'map', 'odom', seq + 400))           # SLAM, its own counter
        obs.append(('/tf', 'base_link', 'left_wheel', seq))     # robot_state_publisher
        obs.append(('/tf', 'base_link', 'right_wheel', seq))    # same message, same number
    obs.append(('/tf_static', 'base_link', 'laser_link', 1))
    assert find_violations(obs) == []


def test_two_publishers_of_one_transform_are_caught():
    obs = []
    for seq in range(1, 20):
        obs.append(('/tf', 'odom', 'base_link', seq))           # EKF
        obs.append(('/tf', 'odom', 'base_link', seq + 3))       # a second publisher
    problems = find_violations(obs)
    assert len(problems) == 1 and 'odom -> base_link' in problems[0]


def test_two_parents_and_mixed_topics_are_caught():
    assert any('more than one parent' in p for p in find_violations(
        [('/tf', 'odom', 'base_link', 1), ('/tf', 'map', 'base_link', 1)]))
    assert any('both /tf and /tf_static' in p for p in find_violations(
        [('/tf', 'map', 'odom', 1), ('/tf_static', 'map', 'odom', 1)]))


# ---------------- map score ----------------
def _write_map(tmp_path, name, cells, origin, resolution=0.05):
    """cells: array of map_score classes, row 0 at the top."""
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[cells == map_score.OCCUPIED] = 0
    pixels[cells == map_score.FREE] = 254
    Image.fromarray(pixels).save(tmp_path / f'{name}.png')
    meta = {'image': f'{name}.png', 'resolution': resolution, 'origin': [origin[0], origin[1], 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    (tmp_path / f'{name}.yaml').write_text(yaml.safe_dump(meta))
    return map_score.load_map(str(tmp_path / f'{name}.yaml'))


def _room(shift_cols=0):
    cells = np.full((40, 40), map_score.FREE, dtype=np.int8)
    cells[10:30, 20 + shift_cols] = map_score.OCCUPIED   # one wall
    return cells


def test_identical_maps_score_one(tmp_path):
    truth = _write_map(tmp_path, 'truth', _room(), (-1.0, -1.0))
    slam = _write_map(tmp_path, 'slam', _room(), (-1.0, -1.0))
    s = map_score.score(truth, slam)
    assert s['iou'] == 1.0 and s['accuracy'] == 1.0 and s['known_cells'] == 1600


def test_shifted_wall_scores_zero_iou(tmp_path):
    truth = _write_map(tmp_path, 'truth', _room(), (-1.0, -1.0))
    slam = _write_map(tmp_path, 'slam', _room(shift_cols=2), (-1.0, -1.0))
    s = map_score.score(truth, slam)
    assert s['iou'] == 0.0
    assert math.isclose(s['accuracy'], 1.0 - 40 / 1600)


def test_origin_offset_is_honoured(tmp_path):
    """The same wall stored with a different image origin must still line up."""
    truth = _write_map(tmp_path, 'truth', _room(), (-1.0, -1.0))
    padded = np.full((50, 60), map_score.UNKNOWN, dtype=np.int8)
    padded[5:45, 10:50] = _room()          # 10 extra columns on the left, 5 rows below
    slam = _write_map(tmp_path, 'slam', padded, (-1.5, -1.25))
    assert map_score.score(truth, slam)['iou'] == 1.0


def test_unknown_cells_are_excluded(tmp_path):
    truth = _write_map(tmp_path, 'truth', _room(), (-1.0, -1.0))
    half = _room()
    half[:, :20] = map_score.UNKNOWN
    slam = _write_map(tmp_path, 'slam', half, (-1.0, -1.0))
    s = map_score.score(truth, slam)
    assert s['known_cells'] == 800 and s['iou'] == 1.0 and s['accuracy'] == 1.0


def test_spawn_pose_moves_the_map_frame(tmp_path):
    """A SLAM map expressed relative to a spawn at (0.5, 0) lines up once the spawn is given."""
    truth = _write_map(tmp_path, 'truth', _room(), (-1.0, -1.0))
    slam = _write_map(tmp_path, 'slam', _room(), (-1.5, -1.0))   # same cells, map frame shifted
    assert map_score.score(truth, slam, spawn=(0.5, 0.0, 0.0))['iou'] == 1.0
    assert map_score.score(truth, slam, spawn=(0.0, 0.0, 0.0))['iou'] < 1.0


# ---------------- trajectory ----------------
def test_ate_of_a_constant_offset():
    pytest.importorskip('evo')
    log = trajectory.TrajectoryLog()
    for i in range(50):
        t = i * 0.1
        true = (t, math.sin(t), 0.0)
        log.add(t, (true[0] + 0.3, true[1] + 0.4, 0.0), true)
    result = trajectory.ate(log)
    assert math.isclose(result['rmse'], 0.5, abs_tol=1e-6)        # sqrt(0.3^2 + 0.4^2)
    assert result['rmse_aligned'] < 1e-6                          # a rigid shift aligns away
    assert result['n'] == 50


def test_map_to_world():
    x, y, yaw = trajectory.map_to_world((1.0, 0.0, 0.0), (2.0, 3.0, math.pi / 2))
    assert math.isclose(x, 2.0, abs_tol=1e-9) and math.isclose(y, 4.0)
    assert math.isclose(yaw, math.pi / 2)


# ---------------- drive files ----------------
def test_drive_file_round_trip(tmp_path):
    rows = [(0.0, 0.5, 0.0), (1.25, 0.0, 1.0), (2.5, 0.0, 0.0)]
    path = tmp_path / 'drives' / 'd.csv'
    save_drive(str(path), rows, comment='test drive')
    assert load_drive(str(path)) == rows
