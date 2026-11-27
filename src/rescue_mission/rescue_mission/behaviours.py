"""py_trees behaviours of the mission tree (docs/PLAN.md, stage 7).

Root: a priority selector, re-evaluated at every tick, children in this order:

  TimeLimit  the sim-time limit is reached           -> Finish (DONE)
  Recover    a recovery is pending (stuck, jump,     -> clear costmaps / back up / re-scan /
             kidnapped)                                 global relocalization (relocalize.py)
  Verify     a pin is waiting for a closer look      -> drive to a viewpoint, dwell
  Explore    ask the frontier selector, drive there  (a failed goal is recorded, not fatal)
  Finish     nothing left to do                      -> DONE, or FAILED with a reason

The decisions (blacklist, counters, which pin next) are in logic.py; these classes move data between
that logic, the frontier selector service and Nav2's actions.
"""
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rescue_msgs.msg import MissionState, SurvivorArray
from rescue_msgs.srv import SelectFrontier as SelectFrontierSrv
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph, Pause, Reset, SerializePoseGraph
import tf2_ros

from rescue_mission import logic as lg
from rescue_mission import relocalize as rl
from rescue_mission.viewpoints import choose_viewpoint

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

DEFAULT_PARAMS = {
    'time_limit_s': 900.0,           # sim seconds; then the mission ends as DONE
    'tick_period_ms': 200,
    'reselect_delay_s': 2.0,         # wait this long before asking again after an empty answer
    'progress_timeout_s': 30.0,      # a goal with no progress for this long is a stuck robot ...
    'progress_min_m': 0.3,           # ... "progress" meaning at least this much closer
    'goal_timeout_s': 150.0,         # hard limit per goal
    'repeat_radius_m': 1.0,          # a goal this close to a reached one is blacklisted
    'confirm_empty': 3,              # empty selector answers in a row before the mission is DONE
    'confirm_unreachable': 8,        # "frontiers but no path" answers in a row before DONE
    'max_consecutive_failures': 8,   # failed goals in a row before the mission is FAILED
    'max_consecutive_recoveries': 6,  # recoveries in a row without a reached goal: FAILED
    'recovery_enabled': True,
    'jump_check_s': 0.5,             # map -> odom is sampled this often (sim seconds) ...
    'jump_distance_m': 0.5,          # ... a step larger than this ...
    'jump_yaw_rad': 0.4,             # ... or this is a localization jump
    'backup_m': 0.3,
    'backup_speed_mps': 0.1,
    'rescan_rad': 6.283,             # spin after a jump, to give SLAM a full scan to relocalize on
    'verify_enabled': False,         # drive to pins for a close look (needs localizer verification)
    'verify_min_hits': 12,           # a candidate needs this many hits to be worth a visit
    'verify_max_attempts': 2,
    'verify_dwell_s': 6.0,           # sim seconds spent at the viewpoint
    'verify_radii_m': [1.5, 1.75, 2.0],
    # kidnap detection and global relocalization (relocalize.py, docs/PHASE10.md)
    'relocalization_enabled': False,
    'kidnap_check_s': 0.5,           # the scan is scored against the map this often (sim seconds)
    'kidnap_low_score': 0.35,        # a score below this is a "low" check ...
    'kidnap_checks': 3,              # ... and this many low checks in a row is a kidnap
    'kidnap_good_score': 0.5,        # the map snapshot used for the search is the last one scoring this
    'kidnap_min_known': 30,          # scan endpoints on known cells needed to score at all
    'kidnap_min_area_m2': 20.0,      # no detection before the map has this much known area
    'reloc_hit_dist_m': 0.15,        # scale of the scan-to-wall distance score
    'reloc_min_score': 0.85,         # the best pose must score at least this ...
    'reloc_min_margin': 0.15,        # ... and beat the best pose elsewhere (>= 1 m away) by this
    # lost mode: the search failed (e.g. kidnapped into unmapped space). SLAM restarts on a fresh map,
    # the robot keeps exploring, and the scan is searched against the saved pre-kidnap map until two
    # confident matches agree with the odometry between them; then the saved map is reloaded there.
    'lost_mode_enabled': False,
    'lost_check_s': 2.0,             # search against the saved map this often (sim seconds)
    'lost_min_known': 40,            # endpoints on known cells of the saved map needed for a match
    'lost_min_motion_m': 1.0,        # two matches must be this far apart by odometry ...
    'lost_min_turn_rad': 0.785,      # ... or turned this much ...
    'lost_tol_m': 0.3,               # ... and their relative pose must equal the odometry change
    'lost_tol_rad': 0.175,           # within these
    'lost_match_window_s': 60.0,     # confident matches older than this are forgotten
    'lost_grace_s': 120.0,           # while lost, "no frontier" in this time after the reset is not DONE
    'lost_wake_backup_m': 0.4,       # after the reset: back up this far and spin once to start the map
    'reloc_repeat_window_s': 90.0,   # a kidnap this soon after a re-seed goes to lost mode (no single match)
    # every re-seed confirmed by motion: a first confident match is only a candidate; the robot drives
    # reloc_confirm_drive_m and a second confident match must agree with odometry (relocalize.py)
    'reloc_confirm_by_motion': False,
    'reloc_confirm_drive_m': 1.2,
    'reloc_confirm_speed_mps': 0.15,
    # after a localization jump the search could not confirm: below this score at SLAM's own pose
    # (against the last good map) the jump is treated as a kidnap and lost mode starts
    'reloc_jump_verify_score': 0.7,
}

PREKIDNAP_GRAPH = '/tmp/rescue_prekidnap_posegraph'


class Context:
    """Everything the behaviours share: the node, the clients, the decision logic and the state."""

    def __init__(self, node, logic, verify_logic, params):
        self.node = node
        self.logic, self.verify_logic = logic, verify_logic
        self.p = {**DEFAULT_PARAMS, **params}
        self.select_client = node.create_client(SelectFrontierSrv, 'select_frontier')
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(node, BackUp, 'backup')
        self.spin_client = ActionClient(node, Spin, 'spin')
        self.drive_client = ActionClient(node, DriveOnHeading, 'drive_on_heading')
        self.clear_clients = [
            node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            node.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')]
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.state_pub = node.create_publisher(MissionState, '/mission/state', 10)
        self.map_msg, self.survivors_msg = None, None
        node.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED)
        node.create_subscription(SurvivorArray, '/survivors', self._on_survivors, LATCHED)
        self.goal = None                 # PoseStamped chosen by SelectFrontier
        self.view_goal, self.view_track = None, None   # verification viewpoint and its pin id
        self.finish = None               # (state, reason) once the Explore branch has ended
        self.t0 = None                   # sim time of the first tick
        self.recovery = None             # reason of the pending recovery, or None
        self.active_handle = None        # Nav2 goal handle being followed
        self._last = (None, None)
        self._mo, self._mo_t = None, -1e9
        # kidnap detection / relocalization
        self.scan_msg, self.sensor_pose = None, None
        self._field, self._field_stamp, self._kid_t = None, None, -1e9
        self.slam_paused, self.relocalized = False, False
        self.graph_saved = False         # the pre-kidnap pose graph is in PREKIDNAP_GRAPH
        self.lost, self.lost_snapshot, self._lost_t, self.lost_since = False, None, -1e9, -1e9
        self.last_reseed_t = -1e9        # sim time of the last SLAM re-seed (direct or from lost mode)
        self.reloc_candidate = None      # (match pose, odom pose) waiting for a confirming match
        self.lost_matches = []           # [(sim time, match pose, odom pose)] confident, while lost
        self.lost_accept = None          # (match pose, odom pose) that passed the motion check
        self.detector = rl.KidnapDetector(
            self.p['kidnap_low_score'], int(self.p['kidnap_checks']), int(self.p['kidnap_min_known']),
            self.p['kidnap_min_area_m2'], self.p['kidnap_good_score'])
        if self.p['relocalization_enabled']:
            node.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
            self.pause_client = node.create_client(Pause, '/slam_toolbox/pause_new_measurements')
            self.serialize_client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
            self.deserialize_client = node.create_client(DeserializePoseGraph,
                                                         '/slam_toolbox/deserialize_map')
            self.reset_client = node.create_client(Reset, '/slam_toolbox/reset')

    def save_graph(self):
        """Serialize slam_toolbox's pose graph (once per kidnap episode, while SLAM is paused)."""
        if not self.graph_saved and self.serialize_client.service_is_ready():
            req = SerializePoseGraph.Request()
            req.filename = PREKIDNAP_GRAPH
            self.serialize_client.call_async(req)
            self.graph_saved = True
            self.node.get_logger().info('pre-kidnap pose graph saved')

    def odom_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time(), Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def check_lost(self, now):
        """While lost: search the saved map; request 'lost_found' once two confident matches agree
        with the odometry between them."""
        if now - self._lost_t < self.p['lost_check_s'] or self.recovery is not None:
            return
        self._lost_t = now
        pts, odom = self.scan_points(), self.odom_pose()
        if pts is None or odom is None or self.lost_snapshot is None:
            return
        res = rl.global_search(self.lost_snapshot, pts, hit_dist_m=self.p['reloc_hit_dist_m'],
                               min_known=int(self.p['lost_min_known']))
        if res is None or res['score'] < self.p['reloc_min_score'] or res['margin'] < self.p['reloc_min_margin']:
            return
        self.lost_matches = [m for m in self.lost_matches if now - m[0] <= self.p['lost_match_window_s']]
        for _t, pose, od in self.lost_matches:
            ok, why = rl.motion_consistent(pose, od, res['pose'], odom, self.p['lost_min_motion_m'],
                                           self.p['lost_min_turn_rad'], self.p['lost_tol_m'],
                                           self.p['lost_tol_rad'])
            if ok:
                self.node.get_logger().info(
                    f'lost: match ({res["pose"][0]:.2f}, {res["pose"][1]:.2f}) score {res["score"]:.2f} '
                    f'confirmed by an earlier match: {why}')
                self.lost_accept = (res['pose'], odom)
                self.request_recovery('lost_found')
                self.cancel_active_goal()
                return
        self.node.get_logger().info(
            f'lost: candidate match ({res["pose"][0]:.2f}, {res["pose"][1]:.2f}) score {res["score"]:.2f}, '
            f'next best elsewhere {res["second"]:.2f}; waiting for a second match after the robot moves')
        self.lost_matches.append((now, res['pose'], odom))

    def _on_scan(self, msg):
        self.scan_msg = msg

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_survivors(self, msg):
        self.survivors_msg = msg

    def now_s(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(), Duration(seconds=0.2))
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def set_state(self, state, reason):
        """Publish /mission/state when the state or the reason changed."""
        if (state, reason) == self._last:
            return
        self._last = (state, reason)
        msg = MissionState()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.state, msg.reason = state, reason
        msg.goals_sent, msg.goals_failed = self.logic.goals_sent, self.logic.goals_failed
        msg.recoveries, msg.verifications = self.logic.recoveries, self.verify_logic.visits
        self.state_pub.publish(msg)
        self.node.get_logger().info(f'mission state {state}: {reason}')

    def request_recovery(self, reason):
        if self.recovery is None:
            self.recovery = reason

    def cancel_active_goal(self):
        if self.active_handle is not None:
            self.active_handle.cancel_goal_async()

    def slam_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(), Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def scan_points(self):
        """Latest scan endpoints in the base_link frame, or None."""
        s = self.scan_msg
        if s is None:
            return None
        if self.sensor_pose is None:
            try:
                t = self.tf_buffer.lookup_transform('base_link', s.header.frame_id, Time(),
                                                    Duration(seconds=0.1))
            except tf2_ros.TransformException:
                return None
            q = t.transform.rotation
            self.sensor_pose = (t.transform.translation.x, t.transform.translation.y,
                                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return rl.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                              self.sensor_pose)

    def field(self):
        """relocalize.Field of the latest /map (rebuilt only when the map changes)."""
        if self.map_msg is None:
            return None
        stamp = (self.map_msg.header.stamp.sec, self.map_msg.header.stamp.nanosec)
        if stamp != self._field_stamp:
            self._field, self._field_stamp = rl.Field.from_msg(self.map_msg), stamp
        return self._field

    def set_slam_paused(self, paused):
        """slam_toolbox's pause service is a toggle: call it only when the state must change."""
        if paused != self.slam_paused and self.pause_client.service_is_ready():
            self.pause_client.call_async(Pause.Request())
            self.slam_paused = paused
            self.node.get_logger().info(f'slam_toolbox new measurements {"paused" if paused else "resumed"}')

    def check_kidnap(self, now):
        """Score the scan at the SLAM pose; pause SLAM on a low score, request a recovery on a kidnap."""
        if now - self._kid_t < self.p['kidnap_check_s'] or self.recovery is not None:
            return
        self._kid_t = now
        f, pts, pose = self.field(), self.scan_points(), self.slam_pose()
        if f is None or pts is None or pose is None:
            return
        kidnapped, s, n = self.detector.update(f, pts, pose, self.p['reloc_hit_dist_m'])
        if self.detector.low_count > 0:
            # stop slam_toolbox from drawing the scans at a wrong pose while we find out, and keep
            # the map as it was (lost mode reloads it)
            self.set_slam_paused(True)
            if self.p['lost_mode_enabled']:
                self.save_graph()
        elif self.slam_paused:
            self.set_slam_paused(False)
            self.graph_saved = False
        if kidnapped:
            self.node.get_logger().warning(
                f'kidnapped: scan-to-map score {s:.2f} ({n} endpoints) for '
                f'{self.detector.low_count} checks in a row')
            self.request_recovery('kidnapped')
            self.cancel_active_goal()

    def poll(self, _tree=None):
        """Runs before every tick: watches map -> odom for a localization jump, and (when enabled)
        the scan-to-map score for a kidnap."""
        if not self.p['recovery_enabled']:
            return
        now = self.now_s()
        if self.lost:
            # the SLAM map is a fresh one with an unknown offset to the saved map: kidnap and jump
            # checks against it mean nothing; only the search against the saved map does
            self.check_lost(now)
            self._mo = None
            return
        if self.p['relocalization_enabled']:
            self.check_kidnap(now)
        if now - self._mo_t < self.p['jump_check_s']:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', Time(), Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._mo is not None:
            step = math.hypot(tr.x - self._mo[0], tr.y - self._mo[1])
            dyaw = (yaw - self._mo[2] + math.pi) % (2 * math.pi) - math.pi
            if step > self.p['jump_distance_m'] or abs(dyaw) > self.p['jump_yaw_rad']:
                self.node.get_logger().warning(
                    f'localization jump: map->odom moved {step:.2f} m and {dyaw:.2f} rad in '
                    f'{now - self._mo_t:.1f} s')
                self.request_recovery('localization_jump')
                self.cancel_active_goal()
        self._mo, self._mo_t = (tr.x, tr.y, yaw), now


# ---------------------------------------------------------------------------------------------
# Explore
# ---------------------------------------------------------------------------------------------
class SelectFrontier(py_trees.behaviour.Behaviour):
    """Ask the frontier selector for a goal. RUNNING while waiting, SUCCESS with a goal,
    FAILURE when exploration is over (ctx.finish says why)."""

    def __init__(self, ctx):
        super().__init__('SelectFrontier')
        self.ctx = ctx
        self.future = None
        self.next_try = 0.0

    def initialise(self):
        self.future = None

    def update(self):
        c = self.ctx
        if self.future is None:
            if c.now_s() < self.next_try or not c.select_client.service_is_ready():
                return Status.RUNNING
            req = SelectFrontierSrv.Request()
            req.blacklist = [Point(x=float(x), y=float(y)) for x, y in c.logic.blacklist]
            self.future = c.select_client.call_async(req)
            return Status.RUNNING
        if not self.future.done():
            return Status.RUNNING
        res, self.future = self.future.result(), None
        xy = (res.goal.pose.position.x, res.goal.pose.position.y) if res.found else None
        decision = c.logic.on_selection(res.found, res.reason, xy)
        if decision == lg.GO:
            c.goal = res.goal
            self.feedback_message = (f'goal ({xy[0]:.1f}, {xy[1]:.1f}) score {res.score:.2f}, '
                                     f'gain {res.gain_m2:.1f} m2, path {res.path_length_m:.1f} m')
            c.node.get_logger().info(self.feedback_message)
            return Status.SUCCESS
        if decision == lg.RETRY:
            self.next_try = 0.0
        elif decision == lg.WAIT:
            self.next_try = c.now_s() + c.p['reselect_delay_s']
            if c.logic.want_clear:
                c.logic.want_clear = False
                c.request_recovery('no_path')
        elif decision == lg.DONE and c.lost and c.now_s() - c.lost_since < c.p['lost_grace_s']:
            # a fresh SLAM map after a lost-mode reset has no frontiers until the robot has moved:
            # move to grow it instead of ending the mission
            c.logic.empty_count = 0
            c.logic.consecutive_recoveries = 0
            self.next_try = c.now_s() + c.p['reselect_delay_s']
            c.request_recovery('lost_wake')
        elif decision == lg.DONE:
            c.finish = ('DONE', c.logic.done_reason)
            return Status.FAILURE
        elif decision == lg.FAILED:
            c.finish = ('FAILED', c.logic.fail_reason)
            return Status.FAILURE
        return Status.RUNNING


class Navigate(py_trees.behaviour.Behaviour):
    """Drive to a PoseStamped with Nav2 and follow the goal. Subclasses say which pose and what to
    do when it ends. A goal with no progress for a while is a stuck robot: a recovery is requested
    and the goal fails."""

    def __init__(self, name, ctx):
        super().__init__(name)
        self.ctx = ctx

    # -- hooks
    def pose(self):
        raise NotImplementedError

    def ended(self, ok, why):
        """Goal over. Returns the Status of this behaviour."""
        raise NotImplementedError

    # -- machinery
    def initialise(self):
        self.goal_pose = self.pose()
        self.goal_xy = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        self.send_future, self.result_future, self.handle = None, None, None
        self.started = self.ctx.now_s()
        self.best_dist, self.best_t = math.inf, self.started
        self.over = False

    def _end(self, ok, why):
        self.over = True
        self.ctx.active_handle = None
        self.feedback_message = why
        text = f'goal ({self.goal_xy[0]:.1f}, {self.goal_xy[1]:.1f}): {why}'
        # rclpy forbids one call site using two severities, hence the two calls
        if ok:
            self.ctx.node.get_logger().info(text)
        else:
            self.ctx.node.get_logger().warning(text)
        return self.ended(ok, why)

    def update(self):
        c = self.ctx
        if self.over:
            return self.ended_status
        status = self._step()
        if status is not None:
            self.ended_status = status
        return status if status is not None else Status.RUNNING

    def _step(self):
        c = self.ctx
        if self.send_future is None:
            if not c.nav_client.server_is_ready():
                return None
            goal = NavigateToPose.Goal()
            goal.pose = self.goal_pose
            self.send_future = c.nav_client.send_goal_async(goal)
            self.sent()
            return None
        if self.handle is None:
            if not self.send_future.done():
                return None
            self.handle = self.send_future.result()
            if not self.handle.accepted:
                return self._end(False, 'rejected by Nav2')
            c.active_handle = self.handle
            self.result_future = self.handle.get_result_async()
        if self.result_future.done():
            code = self.result_future.result().status
            if code == GoalStatus.STATUS_SUCCEEDED:
                return self._end(True, 'reached')
            return self._end(False, f'Nav2 ended with status {code}')
        now = c.now_s()
        xy = c.robot_xy()
        if xy is not None:
            d = math.hypot(self.goal_xy[0] - xy[0], self.goal_xy[1] - xy[1])
            if d < self.best_dist - c.p['progress_min_m']:
                self.best_dist, self.best_t = d, now
        if now - self.best_t > c.p['progress_timeout_s']:
            self.handle.cancel_goal_async()
            c.request_recovery('stuck')
            return self._end(False, f'no progress for {c.p["progress_timeout_s"]:.0f} s (stuck)')
        if now - self.started > c.p['goal_timeout_s']:
            self.handle.cancel_goal_async()
            return self._end(False, f'took longer than {c.p["goal_timeout_s"]:.0f} s')
        return None

    def sent(self):
        pass

    def terminate(self, new_status):
        # preempted by a higher-priority branch while Nav2 was still driving
        if new_status == Status.INVALID and getattr(self, 'handle', None) is not None \
                and not getattr(self, 'over', True) and self.handle.accepted:
            self.handle.cancel_goal_async()
            self.ctx.active_handle = None


class GoToFrontier(Navigate):
    """SUCCESS when reached; FAILURE when it is not (the failure is recorded in the logic)."""

    def __init__(self, ctx):
        super().__init__('GoToFrontier', ctx)

    def pose(self):
        return self.ctx.goal

    def sent(self):
        c = self.ctx
        c.logic.on_goal_sent()
        c.set_state('EXPLORING', f'goal {c.logic.goals_sent} at '
                                 f'({self.goal_xy[0]:.1f}, {self.goal_xy[1]:.1f})')

    def ended(self, ok, why):
        self.ctx.logic.on_result(self.goal_xy, ok)
        return Status.SUCCESS if ok else Status.FAILURE


# ---------------------------------------------------------------------------------------------
# Verify
# ---------------------------------------------------------------------------------------------
def grid_of(msg):
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


class HasVerifyTarget(py_trees.behaviour.Behaviour):
    """SUCCESS when a pin needs a closer look and a viewpoint for it exists (stored in ctx)."""

    def __init__(self, ctx):
        super().__init__('HasVerifyTarget')
        self.ctx = ctx

    def update(self):
        c = self.ctx
        if c.survivors_msg is None or c.map_msg is None:
            return Status.FAILURE
        xy = c.robot_xy()
        if xy is None:
            return Status.FAILURE
        tracks = [(s.id, s.pose.position.x, s.pose.position.y, s.state, s.hit_count)
                  for s in c.survivors_msg.survivors]
        target = c.verify_logic.pick(tracks, xy)
        if target is None:
            return Status.FAILURE
        info = c.map_msg.info
        view = choose_viewpoint(grid_of(c.map_msg), info.resolution,
                                (info.origin.position.x, info.origin.position.y),
                                (target[1], target[2]), xy, tuple(c.p['verify_radii_m']))
        if view is None:
            c.node.get_logger().warning(f'no viewpoint for pin {target[0]}; giving up on it')
            c.verify_logic.on_visit(target[0])
            return Status.FAILURE
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = float(view[0]), float(view[1])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(view[2] / 2), math.cos(view[2] / 2)
        c.view_goal, c.view_track = pose, target[0]
        return Status.SUCCESS


class VisitViewpoint(Navigate):
    """Drive to the viewpoint, then stay verify_dwell_s sim seconds while the localizer collects
    close-range views of the pin. Always ends with SUCCESS: the visit counts as an attempt whether
    or not it worked, so one unreachable pin cannot hold the mission up."""

    def __init__(self, ctx):
        super().__init__('VisitViewpoint', ctx)
        self.dwell_until = None

    def initialise(self):
        self.track_id = self.ctx.view_track
        self.dwell_until = None
        super().initialise()

    def pose(self):
        return self.ctx.view_goal

    def sent(self):
        c = self.ctx
        c.set_state('VERIFYING', f'pin {self.track_id}: viewpoint ({self.goal_xy[0]:.1f}, '
                                 f'{self.goal_xy[1]:.1f})')

    def ended(self, ok, why):
        c = self.ctx
        if ok:
            self.dwell_until = c.now_s() + c.p['verify_dwell_s']
            return None                      # keep going: dwell next
        c.verify_logic.on_visit(self.track_id)
        return Status.SUCCESS

    def update(self):
        c = self.ctx
        if self.dwell_until is not None:
            if c.now_s() < self.dwell_until:
                return Status.RUNNING
            c.verify_logic.on_visit(self.track_id)
            return Status.SUCCESS
        status = super().update()
        # a successful arrival returns None from ended(): stay RUNNING for the dwell
        return Status.RUNNING if status is None else status


# ---------------------------------------------------------------------------------------------
# Recover
# ---------------------------------------------------------------------------------------------
class HasRecovery(py_trees.behaviour.Behaviour):

    def __init__(self, ctx):
        super().__init__('HasRecovery')
        self.ctx = ctx

    def update(self):
        return Status.SUCCESS if self.ctx.recovery else Status.FAILURE


class _Step:
    """One recovery step. start() was called by the constructor; done() polls."""

    def __init__(self, ctx, timeout_s):
        self.ctx, self.timeout_s, self.t0 = ctx, timeout_s, ctx.now_s()

    def timed_out(self):
        return self.ctx.now_s() - self.t0 > self.timeout_s


class ClearCostmaps(_Step):
    def __init__(self, ctx):
        super().__init__(ctx, 5.0)
        self.futures = [cl.call_async(ClearEntireCostmap.Request())
                        for cl in ctx.clear_clients if cl.service_is_ready()]

    def done(self):
        return all(f.done() for f in self.futures) or self.timed_out()


class ActionStep(_Step):
    def __init__(self, ctx, client, goal, timeout_s):
        super().__init__(ctx, timeout_s)
        self.send, self.result = None, None
        if client.server_is_ready():
            self.send = client.send_goal_async(goal)

    def done(self):
        if self.send is None or self.timed_out():
            return True
        if self.result is None and self.send.done():
            handle = self.send.result()
            if not handle.accepted:
                return True
            self.result = handle.get_result_async()
        return self.result is not None and self.result.done()


class StopRobot(_Step):
    def __init__(self, ctx):
        super().__init__(ctx, 1.0)
        ctx.cancel_active_goal()

    def done(self):
        return self.ctx.now_s() - self.t0 > 1.0


class NoStep(_Step):
    def __init__(self, ctx):
        super().__init__(ctx, 0.0)

    def done(self):
        return True


class ResumeSlam(_Step):
    """Let slam_toolbox take new measurements again (it was paused by the kidnap check).

    After a kidnap that the search could not resolve, with lost mode enabled: enter lost mode instead.
    SLAM is reset to a fresh map (drawing the new area on top of the old map at a wrong offset is what
    made the mission fail before), and the old map is kept as the saved pose graph plus a snapshot."""

    def __init__(self, ctx):
        super().__init__(ctx, 10.0)
        c = ctx
        log = c.node.get_logger()
        self.reset_future = None
        if (not c.relocalized and c.p['lost_mode_enabled'] and c.graph_saved
                and c.reset_client.service_is_ready()
                and (c.recovery == 'kidnapped'
                     or (c.recovery == 'localization_jump' and self.jump_unverified(c)))):
            c.lost_snapshot = c.detector.snapshot or c.field()
            c.lost, c.lost_matches, c.lost_accept, c._lost_t = True, [], None, -1e9
            c.lost_since = c.now_s()
            self.reset_future = c.reset_client.call_async(Reset.Request(pause_new_measurements=False))
            c.detector.reset()
            c.detector.snapshot = None
            c.logic.consecutive_recoveries = 0
            log.warning('lost: no confident match; SLAM reset to a fresh map, searching the saved map '
                        f'every {c.p["lost_check_s"]:.0f} s while exploring')
            return
        c.set_slam_paused(False)
        c.graph_saved = False
        c.detector.reset()

    @staticmethod
    def jump_unverified(c):
        """After a map -> odom jump that the search could not confirm: does the scan still fit the last
        good map at SLAM's new pose? A snap to a wrong place right after a kidnap scored 0.40-0.55 there
        (phase 12, seed 4) and 0.51-0.62 (seed 5, scripts/reloc_offline.py trace) while normal driving
        scores 0.95 (1st percentile 0.66)."""
        f, pts, pose = c.detector.snapshot or c.field(), c.scan_points(), c.slam_pose()
        if f is None or pts is None or pose is None:
            return False
        s, n = rl.score(f, pts, pose, c.p['reloc_hit_dist_m'])
        unverified = n >= int(c.p['kidnap_min_known']) and s < c.p['reloc_jump_verify_score']
        c.node.get_logger().info(f'after the jump: scan scores {s:.2f} at the SLAM pose ({n} endpoints): '
                                 f'{"not verified, treating it as a kidnap" if unverified else "verified"}')
        return unverified

    def done(self):
        # The reset's own pause flag does not undo the earlier pause toggle (tested live,
        # scripts/reset_growth_smoke.sh): once the reset is done, toggle the pause off as well.
        if self.reset_future is None:
            return True
        if not self.reset_future.done() and not self.timed_out():
            return False
        self.ctx.set_slam_paused(False)
        self.reset_future = None
        return True


class PauseSlam(NoStep):
    """Freeze the SLAM map while the robot is being relocalized (and save it for lost mode)."""

    def __init__(self, ctx):
        super().__init__(ctx)
        ctx.set_slam_paused(True)
        if ctx.p['lost_mode_enabled']:
            ctx.save_graph()


class LostFound(_Step):
    """Lost mode found the robot on the saved map: reload the saved pose graph at the robot's pose
    now (the confirmed match moved on by the odometry since), and leave lost mode."""

    def __init__(self, ctx):
        super().__init__(ctx, 30.0)
        c = ctx
        match, odom_then = c.lost_accept
        odom_now = c.odom_pose() or odom_then
        d = rl.relative(odom_then, odom_now)
        cs, sn = math.cos(match[2]), math.sin(match[2])
        self.pose = (match[0] + cs * d[0] - sn * d[1], match[1] + sn * d[0] + cs * d[1],
                     (match[2] + d[2] + math.pi) % (2 * math.pi) - math.pi)
        req = DeserializePoseGraph.Request()
        req.filename = PREKIDNAP_GRAPH
        req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        req.initial_pose = Pose2D(x=self.pose[0], y=self.pose[1], theta=self.pose[2])
        self.future = c.deserialize_client.call_async(req) if c.deserialize_client.service_is_ready() else None

    def done(self):
        c = self.ctx
        if self.future is not None and not self.future.done() and not self.timed_out():
            return False
        if self.future is None or not self.future.done():
            c.node.get_logger().warning('lost: reloading the saved map failed; still lost')
            c.lost_matches, c.lost_accept = [], None
            return True
        c.lost, c.lost_snapshot, c.lost_matches, c.lost_accept = False, None, [], None
        c.graph_saved, c.relocalized, c._mo = False, True, None
        c.last_reseed_t = c.now_s()
        c.detector.reset()
        c.detector.snapshot = None
        c.logic.consecutive_recoveries = 0
        c.node.get_logger().info(f'relocalized from lost mode: saved map reloaded at '
                                 f'({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:.2f})')
        return True


class ConfirmDrive(_Step):
    """Move the robot to a second place to confirm a relocalization candidate: turn towards the most open
    direction of the latest scan (the turn is not evidence: the lidar sees 360 degrees) and drive straight
    reloc_confirm_drive_m. If that was cut short, turn to the most open direction 45-135 degrees to the
    side and drive again, so the two legs add up (backing up would return towards the start). SLAM stays
    paused; Nav2's behaviours check collisions on the local costmap (odom frame, live sensor data only)."""

    def __init__(self, ctx):
        super().__init__(ctx, 120.0)
        p = ctx.p
        self.start = ctx.odom_pose()
        drive = lambda: ActionStep(ctx, ctx.drive_client,  # noqa: E731
                                   _drive_goal(p['reloc_confirm_drive_m'], p['reloc_confirm_speed_mps']), 30.0)
        self.steps = [lambda: self.turn_to_open(0.0, math.pi), drive,
                      lambda: (NoStep(ctx) if self.travelled() >= p['lost_min_motion_m']
                               else self.turn_to_open(math.radians(45), math.radians(135))),
                      lambda: (NoStep(ctx) if self.travelled() >= p['lost_min_motion_m'] else drive())]
        self.current = None

    def turn_to_open(self, min_abs, max_abs):
        """Spin towards the farthest scan endpoint whose bearing is min_abs..max_abs off the heading."""
        pts = self.ctx.scan_points()
        if pts is None or not len(pts):
            return NoStep(self.ctx)
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        sel = (np.abs(ang) >= min_abs) & (np.abs(ang) <= max_abs)
        if not sel.any():
            return NoStep(self.ctx)
        dist = np.where(sel, np.hypot(pts[:, 0], pts[:, 1]), -1.0)
        turn = float(ang[int(np.argmax(dist))])
        if abs(turn) <= 0.2:
            return NoStep(self.ctx)
        return ActionStep(self.ctx, self.ctx.spin_client, _spin_goal(turn), 40.0)

    def travelled(self):
        now = self.ctx.odom_pose()
        if now is None or self.start is None:
            return 0.0
        return math.hypot(now[0] - self.start[0], now[1] - self.start[1])

    def done(self):
        if self.timed_out():
            return True
        if self.current is None:
            if not self.steps:
                self.ctx.node.get_logger().info(f'confirm drive: moved {self.travelled():.2f} m by odometry')
                return True
            self.current = self.steps.pop(0)()
        if self.current.done():
            self.current = None
        return False


class Relocalize(_Step):
    """Global search on the last good map snapshot; on a confident answer, re-seed slam_toolbox there
    (serialize the pose graph, deserialize it with START_AT_GIVEN_POSE). Skipped once relocalized."""

    def __init__(self, ctx, attempt):
        super().__init__(ctx, 30.0)
        self.attempt, self.stage, self.future, self.pose = attempt, 'search', None, None
        if ctx.relocalized:
            self.stage = 'done'

    def done(self):
        c = self.ctx
        if self.stage == 'done' or self.timed_out():
            return True
        log = c.node.get_logger()
        if self.stage == 'search':
            if c.now_s() - self.t0 < 1.0:          # a fresh scan from the stopped robot
                return False
            if (c.recovery == 'kidnapped' and c.p['lost_mode_enabled']
                    and c.now_s() - c.last_reseed_t < c.p['reloc_repeat_window_s']):
                # kidnapped again soon after a re-seed: that re-seed may have been wrong (an alias),
                # so a single match is not trusted now; lost mode needs two that agree with odometry
                log.warning(f'relocalization attempt {self.attempt}: kidnapped again '
                            f'{c.now_s() - c.last_reseed_t:.0f} s after a re-seed, not trusting a single match')
                self.stage = 'done'
                return True
            confirm = c.p['reloc_confirm_by_motion']
            f, pts = c.detector.snapshot or c.field(), c.scan_points()
            kw = {'min_known': int(c.p['lost_min_known'])} if confirm else {}
            res = rl.global_search(f, pts, hit_dist_m=c.p['reloc_hit_dist_m'], **kw) \
                if f is not None and pts is not None else None
            if res is None:
                log.warning(f'relocalization attempt {self.attempt}: nothing to search')
                self.stage = 'done'
                return True
            ok = res['score'] >= c.p['reloc_min_score'] and res['margin'] >= c.p['reloc_min_margin']
            log.info(f'relocalization attempt {self.attempt}: best ({res["pose"][0]:.2f}, {res["pose"][1]:.2f}, '
                     f'{res["pose"][2]:.2f}) score {res["score"]:.2f}, next best elsewhere '
                     f'{res["second"]:.2f}: {"confident" if ok else "not confident"}')
            if confirm:
                # No re-seed on a single match: a first confident match becomes a candidate, the robot
                # drives to another place (ConfirmDrive), and a second confident match must agree with
                # the odometry between them (an alias does not move with the robot).
                odom, cand = c.odom_pose(), c.reloc_candidate
                c.reloc_candidate = None
                if cand is None:
                    if ok and odom is not None:
                        c.reloc_candidate = (res['pose'], odom)
                        log.info(f'relocalization attempt {self.attempt}: candidate, driving to confirm it '
                                 'from another place')
                    self.stage = 'done'
                    return True
                if not ok or odom is None:
                    log.info(f'relocalization attempt {self.attempt}: candidate not confirmed '
                             '(no confident match after the drive)')
                    self.stage = 'done'
                    return True
                good, why = rl.motion_consistent(cand[0], cand[1], res['pose'], odom, c.p['lost_min_motion_m'],
                                                 c.p['lost_min_turn_rad'], c.p['lost_tol_m'], c.p['lost_tol_rad'])
                log.info(f'relocalization attempt {self.attempt}: candidate {"confirmed" if good else "not confirmed"}: '
                         f'{why}')
                if not good:
                    self.stage = 'done'
                    return True
            if not ok or not c.serialize_client.service_is_ready():
                self.stage = 'done'
                return True
            cur = c.slam_pose()
            if cur is not None and math.hypot(cur[0] - res['pose'][0], cur[1] - res['pose'][1]) < 0.5 \
                    and abs((cur[2] - res['pose'][2] + math.pi) % (2 * math.pi) - math.pi) < 0.3:
                log.info('relocalization: SLAM pose already agrees with the search, no re-seed')
                c.relocalized = True
                self.stage = 'done'
                return True
            self.pose = res['pose']
            if c.graph_saved:
                # reload the graph saved at the first low score: the current one can hold nodes
                # SLAM added at the wrong pose right after the kidnap, which it would later loop-close to
                self.graph_file, self.future, self.stage = PREKIDNAP_GRAPH, None, 'serialize'
                return False
            req = SerializePoseGraph.Request()
            req.filename = self.graph_file = '/tmp/rescue_reloc_posegraph'
            self.future, self.stage = c.serialize_client.call_async(req), 'serialize'
            return False
        if self.future is not None and not self.future.done():
            return False
        if self.stage == 'serialize':
            req = DeserializePoseGraph.Request()
            req.filename = self.graph_file
            req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
            req.initial_pose = Pose2D(x=self.pose[0], y=self.pose[1], theta=self.pose[2])
            self.future, self.stage = c.deserialize_client.call_async(req), 'deserialize'
            return False
        # deserialized: SLAM restarts at the new pose. The jump in map -> odom is deliberate,
        # and a successful relocalization counts as progress for the recovery counter.
        c.relocalized = True
        c._mo = None
        c.graph_saved = False
        c.last_reseed_t = c.now_s()
        c.logic.consecutive_recoveries = 0
        log.info(f'relocalized: slam_toolbox re-seeded at ({self.pose[0]:.2f}, {self.pose[1]:.2f}, '
                 f'{self.pose[2]:.2f})')
        self.stage = 'done'
        return True


def plan_recovery(ctx, reason):
    """The steps of a recovery, as factories (each starts its step when called)."""
    p = ctx.p
    clear = lambda: ClearCostmaps(ctx)  # noqa: E731
    if reason == 'no_path':
        return [clear]
    if reason == 'lost_found':
        return [lambda: StopRobot(ctx), lambda: LostFound(ctx), clear]
    wake = [lambda: (ActionStep(ctx, ctx.backup_client, _backup_goal(p['lost_wake_backup_m'],  # noqa: E731
                                                                     p['backup_speed_mps']), 20.0)
                     if ctx.lost else NoStep(ctx)),
            lambda: (ActionStep(ctx, ctx.spin_client, _spin_goal(p['rescan_rad']), 40.0)
                     if ctx.lost else NoStep(ctx))]
    if reason == 'lost_wake':
        return [clear] + wake + [clear]
    if reason == 'kidnapped' or (reason == 'localization_jump' and p['relocalization_enabled']):
        # Search where the scan fits the last good map. A jump of map -> odom is checked the same
        # way: slam_toolbox's own scan matcher can snap to a wrong place right after a kidnap.
        # If there is no confident answer, spin for a different view and search once more; if that
        # fails too, resume SLAM and carry on as before.
        ctx.relocalized, ctx.reloc_candidate = False, None
        spin_if_needed = lambda: (NoStep(ctx) if ctx.relocalized else  # noqa: E731
                                  ActionStep(ctx, ctx.spin_client, _spin_goal(p['rescan_rad']), 40.0))
        if p['reloc_confirm_by_motion']:
            # search; with a candidate: drive and confirm it; without one: spin for a new view, search,
            # and if that gives a candidate: drive and confirm it
            def move(first):
                if ctx.relocalized:
                    return NoStep(ctx)
                if ctx.reloc_candidate is not None:
                    return ConfirmDrive(ctx)
                return spin_if_needed() if first else NoStep(ctx)
            steps = [lambda: StopRobot(ctx), lambda: PauseSlam(ctx), lambda: Relocalize(ctx, 1),
                     lambda: move(True), lambda: Relocalize(ctx, 2), lambda: move(False),
                     lambda: Relocalize(ctx, 3)]
        else:
            steps = [lambda: StopRobot(ctx), lambda: PauseSlam(ctx), lambda: Relocalize(ctx, 1), spin_if_needed,
                     lambda: Relocalize(ctx, 2)]
        # (after ResumeSlam: if lost mode started, move so the fresh SLAM map gets its first nodes)
        return steps + [lambda: ResumeSlam(ctx), clear] + wake + [clear]
    if reason == 'localization_jump':
        spin = lambda: ActionStep(ctx, ctx.spin_client, _spin_goal(p['rescan_rad']), 40.0)  # noqa: E731
        return [lambda: StopRobot(ctx), spin, clear]
    back = lambda: ActionStep(ctx, ctx.backup_client,  # noqa: E731
                              _backup_goal(p['backup_m'], p['backup_speed_mps']), 20.0)
    return [clear, back]


def _spin_goal(angle):
    g = Spin.Goal()
    g.target_yaw = float(angle)
    g.time_allowance = Duration(seconds=30).to_msg()
    return g


def _drive_goal(distance, speed):
    g = DriveOnHeading.Goal()
    g.target = Point(x=abs(float(distance)))
    g.speed = float(speed)
    g.time_allowance = Duration(seconds=25).to_msg()
    return g


def _backup_goal(distance, speed):
    g = BackUp.Goal()
    g.target = Point(x=-abs(float(distance)))
    g.speed = float(speed)
    g.time_allowance = Duration(seconds=15).to_msg()
    return g


class DoRecovery(py_trees.behaviour.Behaviour):
    """Run the steps for the pending recovery, then clear it. Always ends in SUCCESS."""

    def __init__(self, ctx):
        super().__init__('DoRecovery')
        self.ctx = ctx

    def initialise(self):
        c = self.ctx
        self.reason = c.recovery
        self.steps, self.current = plan_recovery(c, self.reason), None
        c.logic.on_recovery()
        c.set_state('RECOVERING', {'no_path': 'no frontier has a path: clearing the costmaps',
                                   'stuck': 'robot stuck: clearing costmaps and backing up',
                                   'localization_jump': 'localization jump: stopping and '
                                                        're-scanning',
                                   'kidnapped': 'kidnapped: scan does not fit the map, searching the '
                                                'map for the robot',
                                   'lost_found': 'lost: found on the saved map, reloading it',
                                   'lost_wake': 'lost: moving to grow the fresh map'
                                   }.get(self.reason, self.reason))

    def update(self):
        c = self.ctx
        if self.current is None:
            if not self.steps:
                c.recovery = None
                c.set_state('EXPLORING', f'recovered from {self.reason}')
                return Status.SUCCESS
            self.current = self.steps.pop(0)()
        if self.current.done():
            self.current = None
        return Status.RUNNING


# ---------------------------------------------------------------------------------------------
# time limit, finish, tree
# ---------------------------------------------------------------------------------------------
class IsTimeUp(py_trees.behaviour.Behaviour):

    def __init__(self, ctx):
        super().__init__('IsTimeUp')
        self.ctx = ctx

    def update(self):
        c = self.ctx
        if c.t0 is None:
            c.t0 = c.now_s()
        return Status.SUCCESS if c.now_s() - c.t0 >= c.p['time_limit_s'] else Status.FAILURE


class Finish(py_trees.behaviour.Behaviour):
    """Publish the final state: the reason the Explore branch ended, or the time limit."""

    def __init__(self, ctx, reason=None):
        super().__init__('Finish')
        self.ctx = ctx
        self.reason = reason

    def update(self):
        c = self.ctx
        state, why = c.finish or ('DONE', self.reason or 'exploration ended')
        c.set_state(state, why)
        return Status.SUCCESS


def build_tree(ctx):
    """The mission tree; Recover and Verify are present when enabled in the parameters."""
    children = [py_trees.composites.Sequence(name='TimeLimit', memory=False, children=[
        IsTimeUp(ctx), Finish(ctx, 'time limit reached')])]
    if ctx.p['recovery_enabled']:
        children.append(py_trees.composites.Sequence(name='Recover', memory=True, children=[
            HasRecovery(ctx), DoRecovery(ctx)]))
    if ctx.p['verify_enabled']:
        children.append(py_trees.composites.Sequence(name='Verify', memory=True, children=[
            HasVerifyTarget(ctx), VisitViewpoint(ctx)]))
    children.append(py_trees.composites.Sequence(name='Explore', memory=True, children=[
        SelectFrontier(ctx),
        py_trees.decorators.FailureIsSuccess(name='GoToFrontier (a failure is recorded)',
                                             child=GoToFrontier(ctx))]))
    children.append(Finish(ctx))
    return py_trees.composites.Selector(name='Mission', memory=False, children=children)
