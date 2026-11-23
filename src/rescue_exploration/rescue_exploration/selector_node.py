"""Own frontier selector (docs/PLAN.md, stage 6).

Service /select_frontier (rescue_msgs/SelectFrontier): finds frontier clusters on /map, scores the
most promising ones with Nav2's ComputePathToPose and returns the best goal.

  score = unknown area within sensor range of the goal / path length
          - hazard_weight * hazard area near the goal

Clusters near a blacklisted goal are skipped. The blacklist belongs to the caller (the mission
tree), so this node keeps no state between calls. /frontiers shows the candidates in RViz and
Foxglove: green = chosen, white = other scored candidates, grey = not scored.
"""
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rescue_msgs.srv import SelectFrontier
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from rescue_exploration import frontiers as fr


def grid_array(msg):
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


class FrontierSelector(Node):

    def __init__(self):
        super().__init__('frontier_selector')
        p = self.declare_parameter
        p('map_topic', 'map')
        p('hazard_topic', 'hazard_map')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('min_cluster_m', 0.5)         # frontier clusters shorter than this are dropped
        p('goal_clearance_m', 0.35)     # the goal cell is at least this far from an obstacle
        p('free_max', 25)               # occupancy values up to this count as free
        p('occupied_min', 65)
        p('gain_radius_m', 3.0)         # "within sensor range": unknown area counted around a goal
        p('min_gain_m2', 0.0)           # candidates with less unknown area than this are not worth a trip
        p('hazard_radius_m', 1.0)
        p('hazard_weight', 2.0)
        p('min_path_m', 1.0)
        p('min_goal_distance_m', 0.5)   # skip frontiers closer than this to the robot
        p('blacklist_radius_m', 1.0)
        p('top_k', 6)                   # clusters with a path that are scored, best estimate first
        p('max_plans', 16)              # planner calls per request, at most
        p('plan_timeout_s', 3.0)        # wall seconds per ComputePathToPose call
        p('mode', 'score')              # 'score', or 'random' for the random-goals baseline
        p('random_seed', 0)             # 0 = a different sequence every run
        p('random_tries', 20)
        p('min_random_distance_m', 2.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._g = g
        self.rng = np.random.default_rng(g('random_seed') or None)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg, self.hazard_msg = None, None
        cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(OccupancyGrid, g('map_topic'), self._on_map, latched,
                                 callback_group=cb)
        self.create_subscription(OccupancyGrid, g('hazard_topic'), self._on_hazard, latched,
                                 callback_group=cb)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(MarkerArray, 'frontiers', 1)
        service_cb = ReentrantCallbackGroup()
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                    callback_group=service_cb)
        self.create_service(SelectFrontier, 'select_frontier', self._on_request,
                            callback_group=service_cb)
        self.get_logger().info('frontier selector ready')

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_hazard(self, msg):
        self.hazard_msg = msg

    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self._g('map_frame'), self._g('base_frame'),
                                                Time(), Duration(seconds=0.5))
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def _plan_length(self, x, y, yaw):
        """Length in metres of Nav2's path to (x, y), or None if there is none."""
        if not self.planner.server_is_ready():
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = self._g('map_frame')
        goal.goal.pose.position.x, goal.goal.pose.position.y = float(x), float(y)
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = (
            math.sin(yaw / 2), math.cos(yaw / 2))
        goal.use_start = False
        deadline = time.monotonic() + float(self._g('plan_timeout_s'))
        send = self.planner.send_goal_async(goal)
        while not send.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.01)
        handle = send.result()
        if not handle.accepted:
            return None
        result = handle.get_result_async()
        while not result.done():
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                return None
            time.sleep(0.01)
        poses = result.result().result.path.poses
        if len(poses) < 2:
            return None
        return sum(math.hypot(b.pose.position.x - a.pose.position.x,
                              b.pose.position.y - a.pose.position.y)
                   for a, b in zip(poses[:-1], poses[1:]))

    def _on_request(self, request, response):
        g = self._g
        response.found, response.reason = False, 'ok'
        if self.map_msg is None:
            response.reason = 'no_map'
            return response
        robot = self._robot_xy()
        if robot is None:
            response.reason = 'no_pose'
            return response
        info = self.map_msg.info
        res, origin = info.resolution, (info.origin.position.x, info.origin.position.y)
        grid = grid_array(self.map_msg)
        if g('mode') == 'random':
            return self._random_goal(grid, res, origin, robot, response)
        hazard = None
        if self.hazard_msg is not None and self.hazard_msg.info.width == info.width \
                and self.hazard_msg.info.height == info.height:
            hazard = grid_array(self.hazard_msg)

        clusters = fr.extract_clusters(grid, res, g('min_cluster_m'), g('goal_clearance_m'),
                                       g('free_max'), g('occupied_min'))
        response.n_clusters = len(clusters)
        black = [(b.x, b.y) for b in request.blacklist]
        candidates = []
        for c in clusters:
            x, y = fr.cell_to_xy(c.goal_cell, res, origin)
            if fr.blacklisted(x, y, black, g('blacklist_radius_m')):
                continue
            d = math.hypot(x - robot[0], y - robot[1])
            if d < g('min_goal_distance_m'):
                continue
            c.x, c.y, c.straight_m = x, y, d
            c.gain_m2 = fr.unknown_area_m2(grid, c.goal_cell, res, g('gain_radius_m'))
            if c.gain_m2 < g('min_gain_m2'):
                continue                          # a sliver of unknown at the map edge
            c.hazard_m2 = fr.hazard_area_m2(hazard, c.goal_cell, res, g('hazard_radius_m'))
            c.estimate = fr.score(c.gain_m2, d, c.hazard_m2, g('min_path_m'), g('hazard_weight'))
            c.score = None
            candidates.append(c)
        response.n_candidates = len(candidates)
        if not clusters:
            response.reason = 'no_frontiers'
            self._publish(clusters, None, res, origin)
            return response
        if not candidates:
            response.reason = 'no_frontiers'      # all of them blacklisted or at the robot's feet
            self._publish(clusters, None, res, origin)
            return response

        # Plan to candidates in order of the straight-line estimate until top_k of them have a
        # path, trying at most max_plans: the best few by estimate may all be behind walls, and
        # stopping there would report "no reachable frontier" while reachable ones exist.
        best, planned = None, 0
        for n, c in enumerate(sorted(candidates, key=lambda c: -c.estimate)):
            if planned >= int(g('top_k')) or n >= int(g('max_plans')):
                break
            yaw = fr.goal_heading(grid, c.goal_cell, c.cells, res)
            length = self._plan_length(c.x, c.y, yaw)
            if length is None:
                continue                          # unreachable on Nav2's costmap
            planned += 1
            c.path_m, c.yaw = length, yaw
            c.score = fr.score(c.gain_m2, length, c.hazard_m2, g('min_path_m'), g('hazard_weight'))
            if best is None or c.score > best.score:
                best = c
        self._publish(clusters, best, res, origin)
        if best is None:
            response.reason = 'no_reachable_frontier'
            return response
        response.found = True
        response.goal.header.frame_id = g('map_frame')
        response.goal.header.stamp = self.get_clock().now().to_msg()
        response.goal.pose.position.x, response.goal.pose.position.y = float(best.x), float(best.y)
        response.goal.pose.orientation.z = math.sin(best.yaw / 2)
        response.goal.pose.orientation.w = math.cos(best.yaw / 2)
        response.score, response.gain_m2 = float(best.score), float(best.gain_m2)
        response.path_length_m = float(best.path_m)
        return response

    def _random_goal(self, grid, res, origin, robot, response):
        """Lower-bound baseline: a random known-free cell at least min_random_distance_m away that
        Nav2 can reach. Never reports that exploration is over (the time limit ends the run)."""
        g = self._g
        clear = fr.clearance_m(grid, res, g('occupied_min'))
        rows, cols = np.nonzero((grid >= 0) & (grid <= g('free_max'))
                                & (clear >= g('goal_clearance_m')))
        response.n_clusters = response.n_candidates = int(rows.size)
        if rows.size == 0:
            response.reason = 'no_frontiers'
            return response
        for _ in range(int(g('random_tries'))):
            k = int(self.rng.integers(rows.size))
            x, y = fr.cell_to_xy((int(rows[k]), int(cols[k])), res, origin)
            if math.hypot(x - robot[0], y - robot[1]) < g('min_random_distance_m'):
                continue
            yaw = float(self.rng.uniform(-math.pi, math.pi))
            length = self._plan_length(x, y, yaw)
            if length is None:
                continue
            response.found = True
            response.goal.header.frame_id = g('map_frame')
            response.goal.header.stamp = self.get_clock().now().to_msg()
            response.goal.pose.position.x, response.goal.pose.position.y = float(x), float(y)
            response.goal.pose.orientation.z, response.goal.pose.orientation.w = (
                math.sin(yaw / 2), math.cos(yaw / 2))
            response.path_length_m = float(length)
            return response
        response.reason = 'no_reachable_frontier'
        return response

    def _publish(self, clusters, best, res, origin):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, c in enumerate(clusters):
            x, y = fr.cell_to_xy(c.goal_cell, res, origin)
            m = Marker()
            m.header.frame_id = self._g('map_frame')
            m.ns, m.id, m.type, m.action = 'frontier', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.1
            m.pose.orientation.w = 1.0
            size = 0.25 + 0.05 * min(c.size_m, 4.0)
            m.scale.x = m.scale.y = m.scale.z = float(size)
            scored = getattr(c, 'score', None) is not None
            if c is best:
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.2
            elif scored:
                m.color.r = m.color.g = m.color.b = 1.0
            else:
                m.color.r = m.color.g = m.color.b = 0.5
            m.color.a = 0.9
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FrontierSelector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
