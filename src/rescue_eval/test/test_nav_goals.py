"""Goal sampling and the live-map free mask, without ROS or Gazebo."""
import math
from types import SimpleNamespace

import numpy as np

from rescue_eval.nav_goals import known_free_mask, sample_goals, summarise


class FakePlanner:
    """A 10 m x 10 m open floor, 1 m cells, row 0 at the top (like auto_drive.Planner)."""

    def reachable(self):
        mask = np.ones((10, 10), dtype=bool)
        mask[:, 5] = False                       # a wall column nothing can stand on
        return mask

    def point(self, row, col):
        return (col + 0.5, 9 - row + 0.5)


def test_goals_respect_distance_separation_and_the_live_map():
    rng = np.random.default_rng(1)
    goals = sample_goals(FakePlanner(), lambda x, y: x < 8.0, 6, rng, (0.5, 0.5), 3.0, 2.0)
    assert 3 <= len(goals) <= 6
    previous = (0.5, 0.5)
    for i, (x, y, yaw) in enumerate(goals):
        assert x < 8.0                                         # live-map filter
        assert abs(x - 5.5) > 1e-9                             # reachability filter
        assert math.hypot(x - previous[0], y - previous[1]) >= 3.0
        assert all(math.hypot(x - g[0], y - g[1]) >= 2.0 for g in goals[:i])
        assert -math.pi <= yaw <= math.pi
        previous = (x, y)


def test_sampling_is_reproducible_for_a_seed():
    a = sample_goals(FakePlanner(), lambda x, y: True, 5, np.random.default_rng(7), (0.5, 0.5), 2.0, 1.0)
    b = sample_goals(FakePlanner(), lambda x, y: True, 5, np.random.default_rng(7), (0.5, 0.5), 2.0, 1.0)
    c = sample_goals(FakePlanner(), lambda x, y: True, 5, np.random.default_rng(8), (0.5, 0.5), 2.0, 1.0)
    assert a == b and a != c


def test_known_free_mask_excludes_cells_near_unknown_and_occupied():
    data = np.zeros((20, 20), dtype=np.int8)
    data[:, 0:3] = -1                       # unknown strip
    data[10, 10] = 100                      # one obstacle
    grid = SimpleNamespace(info=SimpleNamespace(resolution=0.05, height=20, width=20),
                           data=data.flatten().tolist())
    mask = known_free_mask(grid, 0.10)      # 2-cell radius
    assert not mask[5, 3]                   # next to the unknown strip
    assert mask[5, 8]                       # clear of it
    assert not mask[10, 11] and not mask[11, 10]     # next to the obstacle
    assert not mask[0, 10]                  # grid border counts as not known free


def test_summary_counts():
    rows = [{'success': 1, 'recoveries_reported': 0, 'recovery_event_count': 0,
             'stuck_without_recovery': 0},
            {'success': 0, 'recoveries_reported': 2, 'recovery_event_count': 3,
             'stuck_without_recovery': 0},
            {'success': 0, 'recoveries_reported': 0, 'recovery_event_count': 0,
             'stuck_without_recovery': 1}]
    s = summarise(rows)
    assert s['goals'] == 3 and s['succeeded'] == 1 and abs(s['success_rate'] - 1 / 3) < 1e-9
    assert s['recoveries'] == 2 and s['recovery_events'] == 3 and s['stuck_without_recovery'] == 1
