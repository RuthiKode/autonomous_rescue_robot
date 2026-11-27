"""After a localization jump the search could not confirm: lost mode when the scan does not fit the last
good map at SLAM's new pose, a normal resume when it does. rl.score is replaced by scripted answers."""
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


class _Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.calls.append(req)
        return None


class _Detector:
    def __init__(self):
        self.snapshot, self.resets = object(), 0

    def reset(self):
        self.resets += 1


class _Logic:
    consecutive_recoveries = 2


class _Ctx:
    def __init__(self, recovery='localization_jump'):
        self.p = {**bt.DEFAULT_PARAMS, 'lost_mode_enabled': True}
        self.recovery, self.relocalized, self.graph_saved, self.lost = recovery, False, True, False
        self.node, self.detector, self.logic, self.reset_client = _Node(), _Detector(), _Logic(), _Client()
        self.paused = True

    def now_s(self):
        return 100.0

    def field(self):
        return object()

    def scan_points(self):
        return 'scan'

    def slam_pose(self):
        return (1.0, 2.0, 0.0)

    def set_slam_paused(self, v):
        self.paused = v


def _resume(monkeypatch, ctx, score, n):
    monkeypatch.setattr(rl, 'score', lambda *a, **k: (score, n))
    bt.ResumeSlam(ctx)
    return ctx


def test_unverified_jump_enters_lost_mode(monkeypatch):
    ctx = _resume(monkeypatch, _Ctx(), 0.55, 60)
    assert ctx.lost and len(ctx.reset_client.calls) == 1


def test_verified_jump_resumes_slam(monkeypatch):
    ctx = _resume(monkeypatch, _Ctx(), 0.9, 60)
    assert not ctx.lost and not ctx.reset_client.calls and ctx.paused is False


def test_too_few_known_endpoints_resumes_slam(monkeypatch):
    ctx = _resume(monkeypatch, _Ctx(), 0.2, 5)
    assert not ctx.lost and not ctx.reset_client.calls


def test_jump_check_off_by_threshold_zero(monkeypatch):
    ctx = _Ctx()
    ctx.p['reloc_jump_verify_score'] = 0.0
    _resume(monkeypatch, ctx, 0.3, 60)
    assert not ctx.lost


def test_relocalized_jump_is_not_rechecked(monkeypatch):
    ctx = _Ctx()
    ctx.relocalized = True
    _resume(monkeypatch, ctx, 0.1, 60)
    assert not ctx.lost and not ctx.reset_client.calls
