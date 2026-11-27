"""The repeat guard: a kidnap soon after a re-seed must not be resolved by a single search match."""
from rescue_mission.behaviours import DEFAULT_PARAMS, Relocalize


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
    snapshot = None


class _Ctx:
    """The parts of behaviours.Context that Relocalize uses."""

    def __init__(self, now, last_reseed_t):
        self.t = now
        self.p = {**DEFAULT_PARAMS, 'lost_mode_enabled': True}
        self.recovery = 'kidnapped'
        self.relocalized = False
        self.last_reseed_t = last_reseed_t
        self.node = _Node()
        self.detector = _Detector()
        self.searched = False

    def now_s(self):
        return self.t

    def field(self):
        return None

    def scan_points(self):
        self.searched = True          # reached the search: the guard let it through
        return None


def _run(ctx):
    step = Relocalize(ctx, 1)
    ctx.t += 1.5                      # past the 1 s wait for a fresh scan
    assert step.done()
    return step


def test_kidnap_soon_after_a_reseed_skips_the_single_match_search():
    ctx = _Ctx(now=1000.0, last_reseed_t=1000.0 - 30.0)
    _run(ctx)
    assert not ctx.searched
    assert not ctx.relocalized
    assert any('not trusting a single match' in m for m in ctx.node.log.lines)


def test_kidnap_long_after_a_reseed_searches_as_usual():
    ctx = _Ctx(now=1000.0, last_reseed_t=1000.0 - 120.0)
    _run(ctx)
    assert ctx.searched


def test_guard_is_off_without_lost_mode():
    ctx = _Ctx(now=1000.0, last_reseed_t=1000.0 - 30.0)
    ctx.p['lost_mode_enabled'] = False
    _run(ctx)
    assert ctx.searched
