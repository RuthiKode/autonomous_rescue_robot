import math

import numpy as np

from rescue_perception.dataset import (assign_boxes, boxes_from_mask, column_of, sample_poses,
                                       yolo_line)


def test_boxes_join_pieces_and_drop_specks():
    m = np.zeros((100, 100), bool)
    m[20:40, 20:30] = True
    m[20:40, 34:40] = True            # same person, cut by a 4 px gap
    m[80:82, 80:82] = True            # speck
    boxes = boxes_from_mask(m)
    assert len(boxes) == 1 and boxes[0][:4] == (20, 20, 40, 40)


def test_two_separate_people_two_boxes():
    m = np.zeros((100, 200), bool)
    m[10:40, 10:30] = True
    m[10:40, 150:170] = True
    assert len(boxes_from_mask(m)) == 2


def test_yolo_line_normalised():
    assert yolo_line((10, 20, 30, 60), 100, 200) == '0 0.200000 0.200000 0.200000 0.200000'


def test_column_and_assignment():
    fx, cx = 467.8, 320.0
    col, rng, front = column_of(0, 0, 0, 3.0, 0.0, fx, cx)
    assert front and abs(col - cx) < 1e-6 and abs(rng - 3.0) < 1e-9
    assert column_of(0, 0, 0, -1.0, 0.0, fx, cx)[2] is False
    left = column_of(0, 0, 0, 3.0, 1.0, fx, cx)[0]
    assert left < cx                                  # left of the heading = smaller column
    survivors = [{'position': {'x': 3.0, 'y': 1.0}}, {'position': {'x': 3.0, 'y': -1.0}}]
    box_left = (int(left) - 20, 100, int(left) + 20, 300, 500)
    assert assign_boxes([box_left], (0, 0, 0), survivors, fx, cx) == [0]
    assert assign_boxes([(630, 0, 639, 50, 100)], (0, 0, 0), survivors, fx, cx) == [-1]


def test_sample_poses_are_free_and_aim_at_survivors():
    free = np.zeros((100, 100), bool)
    free[10:90, 10:90] = True
    surv = [{'position': {'x': 0.0, 'y': 0.0}}]
    poses = sample_poses(np.random.default_rng(1), free, (-2.5, -2.5), 0.05, surv, 30,
                         min_range=0.5, max_range=1.5)
    assert len(poses) == 30
    assert all(-2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 for x, y, _, _ in poses)
    aimed = [p for p in poses if p[3] == 0]
    assert aimed
    for x, y, yaw, _ in aimed:
        off = (math.atan2(-y, -x) - yaw + math.pi) % (2 * math.pi) - math.pi
        assert abs(off) < 1.2                          # 0.25 rad jitter, so well within this
