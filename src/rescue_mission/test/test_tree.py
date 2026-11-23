"""The mission tree's structure and its time-limit branch, with a real node but no simulator."""
import py_trees
import pytest
import rclpy
from rclpy.node import Node

from rescue_mission.behaviours import Context, DEFAULT_PARAMS, build_tree
from rescue_mission.logic import ExploreLogic, VerifyLogic


def make(verify=False, recovery=True, **extra):
    rclpy.init()
    node = Node('test_mission_tree')
    params = {**DEFAULT_PARAMS, 'time_limit_s': 100.0, 'verify_enabled': verify,
              'recovery_enabled': recovery, **extra}
    return Context(node, ExploreLogic(), VerifyLogic(), params)


@pytest.fixture
def ctx():
    c = make()
    yield c
    c.node.destroy_node()
    rclpy.shutdown()


def test_root_is_a_priority_selector_with_the_planned_order(ctx):
    root = build_tree(ctx)
    assert isinstance(root, py_trees.composites.Selector) and not root.memory
    assert [c.name for c in root.children] == ['TimeLimit', 'Recover', 'Explore', 'Finish']
    assert [c.name for c in root.children[2].children][0] == 'SelectFrontier'


def test_verify_branch_sits_between_recover_and_explore():
    c = make(verify=True)
    try:
        names = [x.name for x in build_tree(c).children]
        assert names == ['TimeLimit', 'Recover', 'Verify', 'Explore', 'Finish']
    finally:
        c.node.destroy_node()
        rclpy.shutdown()


def test_branches_can_be_switched_off():
    c = make(recovery=False)
    try:
        assert [x.name for x in build_tree(c).children] == ['TimeLimit', 'Explore', 'Finish']
    finally:
        c.node.destroy_node()
        rclpy.shutdown()


def test_without_a_selector_service_the_tree_keeps_waiting(ctx):
    root = build_tree(ctx)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=1.0)
    for _ in range(3):
        tree.tick()
    assert root.status == py_trees.common.Status.RUNNING
    assert ctx._last == (None, None)              # nothing published yet


def test_time_limit_ends_the_mission_as_done(ctx):
    root = build_tree(ctx)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=1.0)
    ctx.t0 = ctx.now_s() - 1000.0                 # started long ago
    tree.tick()
    assert root.status == py_trees.common.Status.SUCCESS
    assert ctx._last == ('DONE', 'time limit reached')


def test_a_pending_recovery_takes_priority_over_exploring(ctx):
    root = build_tree(ctx)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=1.0)
    ctx.request_recovery('stuck')
    tree.tick()
    recover = root.children[1]
    assert recover.status == py_trees.common.Status.RUNNING
    assert ctx.logic.recoveries == 1 and ctx._last[0] == 'RECOVERING'
    for _ in range(40):                           # no Nav2 servers: every step finishes at once
        tree.tick()
        if ctx.recovery is None:
            break
    assert ctx.recovery is None and ctx._last[0] == 'EXPLORING'


def test_a_finished_explore_branch_ends_the_mission_with_its_reason(ctx):
    root = build_tree(ctx)
    ctx.finish = ('FAILED', '8 goals failed in a row')
    root.children[-1].update()
    assert ctx._last == ('FAILED', '8 goals failed in a row')


def test_request_recovery_keeps_the_first_reason(ctx):
    ctx.request_recovery('localization_jump')
    ctx.request_recovery('stuck')
    assert ctx.recovery == 'localization_jump'
