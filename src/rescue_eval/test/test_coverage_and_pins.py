"""Coverage against the truth grid and the one-to-one pin scorer."""
import math
import os

import numpy as np
import pytest

from rescue_eval import map_score as ms
from rescue_eval.coverage import coverage, reachable_truth_mask, time_to
from rescue_eval.survivor_score import score_pins


def truth_grid():
    """A 4 m x 4 m room (0.05 m cells) with a wall across the middle and a 1 m gap."""
    cells = np.full((100, 100), ms.UNKNOWN, dtype=np.int8)
    cells[10:90, 10:90] = ms.FREE
    cells[10:90, 49:51] = ms.OCCUPIED               # wall at x = 0
    cells[40:60, 49:51] = ms.FREE                   # the gap
    return {'cells': cells, 'resolution': 0.05, 'origin': (-2.5, -2.5, 0.0)}


def test_reachable_mask_keeps_the_robot_away_from_walls():
    truth = truth_grid()
    mask = reachable_truth_mask(truth, (-1.0, 0.0), robot_radius_m=0.30)
    assert mask.sum() > 0
    assert not mask[10, 50]                          # a cell touching the outer wall
    assert not mask[12, 20]                          # inside 0.30 m of the wall
    assert mask[50, 30] and mask[50, 70]             # both rooms, through the gap
    assert not mask[20, 49]                          # the wall column


def test_a_closed_off_room_is_not_reachable():
    truth = truth_grid()
    truth['cells'][40:60, 49:51] = ms.OCCUPIED       # close the gap
    mask = reachable_truth_mask(truth, (-1.0, 0.0), robot_radius_m=0.30)
    assert mask[50, 30] and not mask[50, 70]


def test_coverage_counts_mapped_free_cells_in_the_reachable_set():
    truth = truth_grid()
    mask = reachable_truth_mask(truth, (-1.0, 0.0), robot_radius_m=0.30)
    slam = {'cells': np.full((100, 100), ms.UNKNOWN, dtype=np.int8), 'resolution': 0.05,
            'origin': (-2.5, -2.5, 0.0)}
    assert coverage(truth, mask, slam) == 0.0
    slam['cells'][truth['cells'] == ms.FREE] = ms.FREE
    assert coverage(truth, mask, slam) == pytest.approx(1.0)
    slam['cells'][:, 50:] = ms.UNKNOWN               # only the left half explored
    c = coverage(truth, mask, slam)
    assert 0.3 < c < 0.7


def test_time_to_level():
    curve = [(0.0, 0.0), (10.0, 0.4), (20.0, 0.91), (30.0, 0.95)]
    assert time_to(curve, 0.9) == 20.0
    assert math.isnan(time_to(curve, 0.99))


def test_pin_scoring_one_to_one():
    truth = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]
    s = score_pins([(0.2, 0.1), (5.3, 0.0), (20.0, 0.0)], truth)
    assert (s['matched'], s['pins'], s['truth']) == (2, 3, 3)
    assert s['precision'] == pytest.approx(2 / 3) and s['recall'] == pytest.approx(2 / 3)
    assert s['duplicate_rate'] == 0.0
    assert s['pin_error_m'] == pytest.approx((math.hypot(0.2, 0.1) + 0.3) / 2)


def test_a_second_pin_on_a_matched_survivor_is_a_duplicate_not_a_match():
    s = score_pins([(0.1, 0.0), (0.6, 0.0)], [(0.0, 0.0)])
    assert s['matched'] == 1 and s['recall'] == 1.0
    assert s['precision'] == 0.5 and s['duplicate_rate'] == 0.5
    assert s['pin_error_m'] == pytest.approx(0.1)        # the closer pin wins


def test_no_pins_gives_nan_precision_and_zero_recall():
    s = score_pins([], [(0.0, 0.0)])
    assert math.isnan(s['precision']) and s['recall'] == 0.0


def test_committed_truth_maps_have_a_reachable_spawn():
    from ament_index_python.packages import get_package_share_directory
    worlds = os.path.join(get_package_share_directory('rescue_simulation'), 'worlds')
    for name in ('rooms', 'collapsed', 'open_plan'):
        truth = ms.load_map(os.path.join(worlds, name + '_truth_map.yaml'))
        mask = reachable_truth_mask(truth, (0.0, 0.0))
        assert mask.sum() * truth['resolution'] ** 2 > 20.0, name       # more than 20 m^2
