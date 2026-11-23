import math

import numpy as np

from rescue_eval.inject import block_pose_ahead, block_sdf, pick_teleport_target


def test_block_is_ahead_of_the_robot_and_shares_its_heading():
    x, y, yaw = block_pose_ahead((1.0, 2.0, math.pi / 2), 1.5)
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.5) < 1e-9 and abs(yaw - math.pi / 2) < 1e-12


def test_block_sdf_is_a_static_box_of_the_requested_size():
    sdf = block_sdf(1.0, 2.0, 0.5, width=1.6, thickness=0.3, height=1.0)
    assert '<static>true</static>' in sdf and '<size>0.3 1.6 1.0</size>' in sdf
    assert '<pose>1.000 2.000 0.50 0 0 0.5000</pose>' in sdf
    assert sdf.count('<box>') == 2                      # collision and visual


def test_teleport_target_is_reachable_and_far():
    mask = np.zeros((100, 100), bool)
    mask[10:90, 10:90] = True
    rng = np.random.default_rng(3)
    for _ in range(20):
        x, y, yaw = pick_teleport_target(mask, 0.1, (-5.0, -5.0), (0.0, 0.0), rng, 4.0)
        assert math.hypot(x, y) >= 4.0
        assert -4.0 <= x <= 4.0 and -4.0 <= y <= 4.0          # inside the reachable square
        assert -math.pi <= yaw <= math.pi


def test_teleport_target_fails_loudly_when_nothing_is_far_enough():
    mask = np.zeros((20, 20), bool)
    mask[5:15, 5:15] = True
    try:
        pick_teleport_target(mask, 0.1, (0.0, 0.0), (1.0, 1.0), np.random.default_rng(0), 50.0)
    except ValueError as exc:
        assert 'far enough' in str(exc)
    else:
        raise AssertionError('expected ValueError')


def _log(errors):
    from rescue_eval.trajectory import TrajectoryLog
    log = TrajectoryLog()
    for i, e in enumerate(errors):
        log.add(float(i), (e, 0.0, 0.0), (0.0, 0.0, 0.0))
    return log


def test_injection_outcome_recovered_when_the_pose_comes_back():
    from rescue_eval.explore_eval import injection_outcome
    inj = {'counts_before': {'goals_sent': 5, 'recoveries': 0}, 'log_index': 0}
    log = _log([0.0] * 50 + [3.0] * 30 + [0.1] * 300)           # jumped, then SLAM recovered
    out = injection_outcome(inj, 'teleport', log, [(0.0, 0.4), (300.0, 0.95)],
                            ('DONE', 'x', 300.0), {'goals_sent': 12, 'recoveries': 1})
    assert out['recovered'] and out['goals_after'] == 7 and out['recoveries_after'] == 1
    assert out['pose_error_peak_m'] == 3.0 and out['pose_error_final_m'] < 0.2


def test_injection_outcome_fails_on_a_lost_pose_or_a_failed_mission():
    from rescue_eval.explore_eval import injection_outcome
    inj = {'counts_before': {}, 'log_index': 0}
    lost = injection_outcome(inj, 'teleport', _log([0.0] * 10 + [2.5] * 300), [(0.0, 0.9)],
                             ('DONE', 'x', 1.0), {})
    assert not lost['recovered']                                 # DONE, but the pose never came back
    blocked = injection_outcome(inj, 'block', _log([0.0] * 100), [(0.0, 0.3)],
                                ('FAILED', 'x', 1.0), {})
    assert not blocked['recovered']
    ok = injection_outcome(inj, 'block', _log([2.5] * 100), [(0.0, 0.9)], ('EXPLORING', 'x', 1.0), {})
    assert ok['recovered']                                       # pose error only matters for a teleport
