"""Lost-mode timeout: after lost_timeout_s without a confirmed match the saved map is given up, the
robot keeps exploring the fresh map, and the final state says so. Context methods run on a stub."""
from rescue_mission import behaviours as bt


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


class _Detector:
    def __init__(self):
        self.snapshot, self.resets = object(), 0

    def reset(self):
        self.resets += 1


class _Ctx:
    lost_timed_out = bt.Context.lost_timed_out
    abandon_saved_map = bt.Context.abandon_saved_map

    def __init__(self, timeout=360.0):
        self.p = {**bt.DEFAULT_PARAMS, 'lost_timeout_s': timeout}
        self.lost, self.lost_since, self.recovery = True, 100.0, None
        self.lost_snapshot, self.lost_matches, self.lost_accept = object(), [(1, 2, 3)], None
        self.lost_abandoned, self.graph_saved, self._mo = False, True, (0, 0, 0)
        self.node, self.detector, self.finish, self.state = _Node(), _Detector(), None, None

    def set_state(self, state, why):
        self.state = (state, why)


def test_not_timed_out_before_the_limit():
    assert not _Ctx().lost_timed_out(100.0 + 359.0)


def test_timed_out_after_the_limit():
    assert _Ctx().lost_timed_out(100.0 + 361.0)


def test_zero_means_search_forever():
    assert not _Ctx(timeout=0.0).lost_timed_out(1e9)


def test_no_timeout_while_a_recovery_runs():
    c = _Ctx()
    c.recovery = 'lost_found'               # a reload in progress must finish
    assert not c.lost_timed_out(100.0 + 1000.0)


def test_abandon_leaves_lost_mode_and_resumes_checks():
    c = _Ctx()
    c.abandon_saved_map(500.0)
    assert not c.lost and c.lost_abandoned
    assert c.lost_snapshot is None and c.lost_matches == [] and not c.graph_saved
    assert c.detector.resets == 1 and c.detector.snapshot is None and c._mo is None
    assert 'giving up on the saved map' in c.node.log.lines[-1]


def test_final_state_reports_the_abandoned_map():
    c = _Ctx()
    c.abandon_saved_map(500.0)
    c.finish = ('DONE', 'no frontier left')
    bt.Finish(c).update()
    assert c.state[0] == 'DONE' and 'map from before the kidnap was abandoned' in c.state[1]


def test_final_state_reports_still_lost():
    c = _Ctx()
    bt.Finish(c, 'time limit reached').update()
    assert c.state == ('DONE', 'time limit reached; still lost: the pose is on a fresh map, '
                               'not on the map from before the kidnap')


def test_final_state_unchanged_without_lost_mode():
    c = _Ctx()
    c.lost = False
    c.finish = ('DONE', 'no frontier left')
    bt.Finish(c).update()
    assert c.state == ('DONE', 'no frontier left')
