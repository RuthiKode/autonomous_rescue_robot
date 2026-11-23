"""Mission behaviour tree node: ticks the tree built in behaviours.py.

  ros2 launch rescue_mission mission.launch.py

Needs Nav2 (navigate_to_pose, compute_path_to_pose, backup, spin), slam_toolbox's /map and the
frontier selector. Publishes /mission/state (rescue_msgs/MissionState) on every state change.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import py_trees_ros

from rescue_mission.behaviours import Context, DEFAULT_PARAMS, build_tree
from rescue_mission.logic import ExploreLogic, VerifyLogic


def main():
    rclpy.init()
    node = Node('mission_tree')
    for key, default in DEFAULT_PARAMS.items():
        node.declare_parameter(key, default)
    params = {k: node.get_parameter(k).value for k in DEFAULT_PARAMS}
    logic = ExploreLogic(params['repeat_radius_m'], params['confirm_empty'],
                         params['max_consecutive_failures'], params['max_consecutive_recoveries'],
                         params['confirm_unreachable'])
    verify = VerifyLogic(params['verify_min_hits'], params['verify_max_attempts'])
    ctx = Context(node, logic, verify, params)
    tree = py_trees_ros.trees.BehaviourTree(root=build_tree(ctx), unicode_tree_debug=False)
    tree.setup(node=node, timeout=15.0)
    ctx.set_state('STARTING', 'waiting for the map, Nav2 and the frontier selector')
    tree.tick_tock(period_ms=int(params['tick_period_ms']), pre_tick_handler=ctx.poll)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            tree.shutdown()               # destroys the node
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
