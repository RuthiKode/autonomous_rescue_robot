"""The world generator must be deterministic and respect the plan's constraints."""
import math
import os

import pytest
import yaml

from rescue_simulation.generate_worlds import generate

PKG = os.path.join(os.path.dirname(__file__), '..')
with open(os.path.join(PKG, 'config', 'world_generator.yaml')) as _f:
    CFG = yaml.safe_load(_f)
WORLDS = list(CFG['worlds'])


def _gen(name, seed=None, **overrides):
    world_cfg = dict(CFG['worlds'][name], **overrides)
    return generate(name, world_cfg, CFG['common'], CFG['seed'] if seed is None else seed)


def test_three_layouts_exist():
    assert {'rooms', 'collapsed', 'open_plan'} <= set(WORLDS)


@pytest.mark.parametrize('name', WORLDS)
def test_same_seed_is_identical_and_other_seed_differs(name):
    assert _gen(name) == _gen(name)
    assert _gen(name, seed=CFG['seed'] + 1)[0] != _gen(name)[0]


@pytest.mark.parametrize('name', WORLDS)
@pytest.mark.parametrize('seed', range(1, 21))
def test_survivors_are_valid_for_many_seeds(name, seed):
    _, truth = _gen(name, seed=seed)
    survivors = truth['survivors']
    assert 3 <= len(survivors) <= 5
    assert {s['posture'] for s in survivors} == {'standing', 'sitting', 'lying'}
    assert any(s['occluded'] for s in survivors)
    assert [s['id'] for s in survivors] == list(range(1, len(survivors) + 1))
    hx, hy = truth['bounds']['size_x'] / 2, truth['bounds']['size_y'] / 2
    clear = CFG['common']['spawn_clear_radius']
    for s in survivors:
        x, y = s['position']['x'], s['position']['y']
        assert abs(x) < hx and abs(y) < hy
        assert math.hypot(x, y) > clear
    assert truth['robot_spawn'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}


def test_gap_width_is_a_parameter():
    sdf_narrow, truth_narrow = _gen('collapsed', gap_width=0.6)
    sdf_wide, truth_wide = _gen('collapsed', gap_width=1.2)
    assert truth_narrow['gap_width'] == 0.6 and truth_wide['gap_width'] == 1.2
    assert sdf_narrow != sdf_wide


def test_impossible_gap_width_is_rejected():
    with pytest.raises(ValueError):
        _gen('collapsed', gap_width=6.0)


@pytest.mark.parametrize('name', WORLDS)
def test_committed_worlds_match_the_generator(name):
    sdf, truth = _gen(name)
    with open(os.path.join(PKG, 'worlds', name + '.sdf')) as f:
        assert f.read() == sdf, 'worlds/ is stale: re-run generate_worlds'
    with open(os.path.join(PKG, 'worlds', name + '_survivors.yaml')) as f:
        assert yaml.safe_load(f) == truth
