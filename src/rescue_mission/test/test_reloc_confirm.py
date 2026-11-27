"""Confirm-by-motion re-seed: no re-seed on a single match; a second confident match after the robot has
driven >= 1 m must agree with odometry. global_search is replaced by scripted answers."""
import math

import pytest

from rescue_mission import behaviours as bt
from rescue_mission import relocalize as rl


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warning = info


class _Node:
    def __init__(self):
        self.log = _Log()

    def get_logger(self):
        return self.log


class _Future:
    def done(self):
        return True

    def result(self):
        return None


class _Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.calls.append(req)
        return _Future()


class _Logic:
    consecutive_recoveries = 3


class _Detector:
    snapshot = object()               # anything non-None: the search is scripted


class _Ctx:
    def __init__(self):
        self.t = 1000.0
        self.p = {**bt.DEFAULT_PARAMS, 'lost_mode_enabled': True, 'reloc_confirm_by_motion': True,
                  'lost_min_turn_rad': 4.0}
        self.recovery = 'kidnapped'
        self.relocalized, self.reloc_candidate = False, None
        self.last_reseed_t, self.graph_saved, self._mo = -1e9, True, (0, 0, 0)
        self.node, self.detector, self.logic = _Node(), _Detector(), _Logic()
        self.serialize_client, self.deserialize_client = _Client(), _Client()
        self.odom = (0.0, 0.0, 0.0)
        self.slam = (20.0, 20.0, 0.0)         # SLAM is far off: a confirmed match means a re-seed

    def now_s(self):
        return self.t

    def field(self):
        return None

    def scan_points(self):
        return 'scan'

    def odom_pose(self):
        return self.odom

    def slam_pose(self):
        return self.slam


def _answer(pose, score=0.97, second=0.5):
    return {'pose': pose, 'score': score, 'second': second, 'margin': score - second, 'n_known': 60,
            'candidates': 1000}


def _search(ctx, attempt, monkeypatch, answer):
    monkeypatch.setattr(rl, 'global_search', lambda *a, **k: answer)
    step = bt.Relocalize(ctx, attempt)
    ctx.t += 1.5
    while not step.done():
        ctx.t += 0.1
    return step


def test_a_single_confident_match_does_not_reseed(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    assert ctx.reloc_candidate is not None
    assert not ctx.deserialize_client.calls and not ctx.relocalized


def test_second_match_that_follows_odometry_reseeds_at_the_second_match(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    ctx.odom = (1.2, 0.0, 0.0)                  # drove 1.2 m straight ahead (odom heading 0)
    second = (3.0 + 1.2 * math.cos(0.5), 1.0 + 1.2 * math.sin(0.5), 0.5)
    _search(ctx, 2, monkeypatch, _answer(second))
    assert ctx.relocalized
    assert len(ctx.deserialize_client.calls) == 1
    req = ctx.deserialize_client.calls[0]
    assert req.initial_pose.x == pytest.approx(second[0]) and req.initial_pose.y == pytest.approx(second[1])
    assert req.filename == bt.PREKIDNAP_GRAPH


def test_an_alias_that_does_not_move_with_the_robot_is_rejected(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    ctx.odom = (1.2, 0.0, 0.0)
    _search(ctx, 2, monkeypatch, _answer((3.0, 1.0, 0.5)))     # same place again: an alias
    assert not ctx.relocalized and not ctx.deserialize_client.calls
    assert ctx.reloc_candidate is None


def test_a_drive_that_stopped_short_does_not_confirm(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    ctx.odom = (0.4, 0.0, 0.0)                  # blocked after 0.4 m
    second = (3.0 + 0.4 * math.cos(0.5), 1.0 + 0.4 * math.sin(0.5), 0.5)
    _search(ctx, 2, monkeypatch, _answer(second))
    assert not ctx.relocalized and not ctx.deserialize_client.calls


def test_a_turn_on_the_spot_does_not_confirm(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    ctx.odom = (0.0, 0.0, 1.57)                 # turned 90 degrees, no translation
    _search(ctx, 2, monkeypatch, _answer((3.0, 1.0, 0.5 + 1.57)))
    assert not ctx.relocalized and not ctx.deserialize_client.calls


def test_confirmed_match_that_agrees_with_slam_is_not_reseeded(monkeypatch):
    ctx = _Ctx()
    _search(ctx, 1, monkeypatch, _answer((3.0, 1.0, 0.5)))
    ctx.odom = (1.2, 0.0, 0.0)
    second = (3.0 + 1.2 * math.cos(0.5), 1.0 + 1.2 * math.sin(0.5), 0.5)
    ctx.slam = second
    _search(ctx, 2, monkeypatch, _answer(second))
    assert ctx.relocalized and not ctx.deserialize_client.calls


def test_motion_check_rejects_a_pure_rotation_with_turn_alternative_off():
    ok, why = rl.motion_consistent((3.0, 1.0, 0.5), (0.0, 0.0, 0.0), (3.0, 1.0, 2.07), (0.0, 0.0, 1.57),
                                   min_turn_rad=4.0)
    assert not ok and 'not enough motion' in why
