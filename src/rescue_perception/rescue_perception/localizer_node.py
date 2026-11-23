"""Survivor localizer: detections to persistent pins on the map.

Subscribes: /detections (vision_msgs/Detection2DArray), /camera/depth (32FC1),
            /camera/camera_info, TF.
Publishes:  /survivors (rescue_msgs/SurvivorArray, transient local),
            /survivors/markers (visualization_msgs/MarkerArray).

Per detection (docs/PLAN.md, stage 5):
 1. match it with the depth frame of the same time (approximate time synchronizer) and
    look up TF at the detection's own timestamp, never at "now";
 2. median depth over the central part of the box;
 3. back-project the box centre with the camera intrinsics, transform it to `map`;
 4. reject invalid or out-of-range depth and low confidence;
 5. associate with the nearest track, confirm after N hits from K viewpoints (tracker.py).
"""
import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
import message_filters
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rescue_msgs.msg import Survivor, SurvivorArray
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from rescue_perception.geometry import (backproject, inverse_transform_point, median_depth,
                                        point_visible, transform_point)
from rescue_perception.tracker import CONFIRMED, Tracker

MERGED = 'merged'      # a confirmed track that a stronger one within merge_radius_m replaces
STATE_COLOUR = {'candidate': (1.0, 0.8, 0.0), 'confirmed': (1.0, 0.1, 0.1),
                MERGED: (0.5, 0.5, 0.5)}


def survivor_messages(tracks, stamp, frame_id, suppressed=()):
    """Build the /survivors message and its RViz markers from tracks.

    Tracks whose id is in `suppressed` are reported with state 'merged', so only 'confirmed'
    ones count as pins. Kept free of the node so a test can build real messages: ROS message
    fields need real floats, and a wrong type aborts the whole process in C (an int scale did
    exactly that).
    """
    msg = SurvivorArray()
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    markers = MarkerArray()
    for t in tracks:
        s = Survivor()
        state = MERGED if t.id in suppressed else t.state
        s.id, s.state = int(t.id), state
        s.pose.position.x, s.pose.position.y = float(t.x), float(t.y)
        s.pose.position.z = float(t.z)
        s.pose.orientation.w = 1.0
        s.confidence, s.hit_count = float(t.confidence), int(t.hits)
        s.first_seen = TimeMsg(sec=int(t.first_seen),
                               nanosec=int((t.first_seen % 1.0) * 1e9))
        msg.survivors.append(s)
        colour = STATE_COLOUR[state]
        for ns, mtype, scale, z_off in (('pin', Marker.SPHERE, (0.35, 0.35, 0.35), 0.0),
                                        ('label', Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.3), 0.5)):
            m = Marker()
            m.header = msg.header
            m.ns, m.id, m.type, m.action = ns, int(t.id), mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y = float(t.x), float(t.y)
            m.pose.position.z = float(max(t.z, 0.2) + z_off)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = (*colour, 0.9)
            if mtype == Marker.TEXT_VIEW_FACING:
                m.color.r = m.color.g = m.color.b = 1.0
                m.text = f'#{t.id} {state} ({t.hits})'
            markers.markers.append(m)
    return msg, markers


def depth_to_array(msg):
    """sensor_msgs/Image 32FC1 to a float32 array (no cv_bridge needed)."""
    arr = np.frombuffer(msg.data, dtype=np.float32)
    return arr.reshape(msg.height, msg.step // 4)[:, :msg.width]


class SurvivorLocalizer(Node):

    def __init__(self):
        super().__init__('survivor_localizer')
        p = self.declare_parameter
        p('detections_topic', '/detections')
        p('depth_topic', '/camera/depth')
        p('camera_info_topic', '/camera/camera_info')
        p('survivors_topic', '/survivors')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('sync_slop_s', 0.15)
        p('min_confidence', 0.30)
        p('central_fraction', 0.5)
        p('min_range_m', 0.4)
        p('max_range_m', 6.0)
        p('association_radius_m', 1.0)
        p('confirm_hits', 5)
        p('confirm_views', 2)
        p('view_separation_m', 0.75)
        p('expire_s', 60.0)
        p('confirm_confidence', 0.0)     # mean confidence a confirmed track needs
        p('merge_radius_m', 0.0)         # only the strongest confirmed track within this is a pin
        p('min_hit_rate', 0.0)           # 0 = off; see Tracker
        p('rate_after_views', 20)
        p('occlusion_tolerance_m', 0.6)
        p('tf_timeout_s', 0.1)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._g = g
        self.tracker = Tracker(g('association_radius_m'), g('confirm_hits'), g('confirm_views'),
                               g('view_separation_m'), g('expire_s'), g('min_hit_rate'),
                               g('rate_after_views'), g('confirm_confidence'),
                               g('merge_radius_m'))
        self._k = None
        self.stats = {'pairs': 0, 'detections': 0, 'used': 0, 'low_confidence': 0,
                      'bad_depth': 0, 'tf_failed': 0}
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(SurvivorArray, g('survivors_topic'), latched)
        self._marker_pub = self.create_publisher(MarkerArray, g('survivors_topic') + '/markers',
                                                 latched)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_info, 1)
        reliable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        det = message_filters.Subscriber(self, Detection2DArray, g('detections_topic'),
                                         qos_profile=reliable)
        depth = message_filters.Subscriber(self, Image, g('depth_topic'), qos_profile=reliable)
        self._sync = message_filters.ApproximateTimeSynchronizer([det, depth], 30,
                                                                 g('sync_slop_s'))
        self._sync.registerCallback(self._on_pair)
        self.create_timer(20.0, self._log_stats)

    def _on_info(self, msg):
        self._k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])      # fx, fy, cx, cy

    def _lookup(self, target, source, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(stamp), Duration(seconds=self._g('tf_timeout_s')))
        except tf2_ros.TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _on_pair(self, detections, depth_msg):
        self.stats['pairs'] += 1
        if self._k is None:
            return
        fx, fy, cx, cy = self._k
        depth = depth_to_array(depth_msg)
        stamp = detections.header.stamp
        cam_to_map = self._lookup(self._g('map_frame'), depth_msg.header.frame_id, stamp)
        base_in_map = self._lookup(self._g('map_frame'), self._g('base_frame'), stamp)
        changed = self._count_views(depth, cam_to_map)   # every frame, with or without detections
        for d in detections.detections:
            self.stats['detections'] += 1
            score = d.results[0].hypothesis.score if d.results else 0.0
            if score < self._g('min_confidence'):
                self.stats['low_confidence'] += 1
                continue
            u, v = d.bbox.center.position.x, d.bbox.center.position.y
            z = median_depth(depth, u, v, d.bbox.size_x, d.bbox.size_y,
                             self._g('central_fraction'), self._g('min_range_m'),
                             self._g('max_range_m'))
            if z is None:
                self.stats['bad_depth'] += 1
                continue
            if cam_to_map is None or base_in_map is None:
                self.stats['tf_failed'] += 1
                continue
            x, y, zz = transform_point(*cam_to_map, backproject(u, v, z, fx, fy, cx, cy))
            known = {t.id: t.state for t in self.tracker.tracks}
            track = self.tracker.update(x, y, zz, score, stamp.sec + stamp.nanosec * 1e-9,
                                        (base_in_map[0][0], base_in_map[0][1]))
            self.stats['used'] += 1
            if track.id not in known:
                self.get_logger().info(f'new candidate {track.id} at ({x:.2f}, {y:.2f}),'
                                       f' depth {z:.2f} m, conf {score:.2f}')
            elif known[track.id] != track.state:
                self.get_logger().info(
                    f'track {track.id} {track.state} at ({track.x:.2f}, {track.y:.2f})'
                    f' after {track.hits} hits from {len(track.views)} viewpoints')
        now = stamp.sec + stamp.nanosec * 1e-9
        for t in self.tracker.expire(now):
            self.get_logger().info(f'candidate {t.id} expired after {t.hits} hit(s)')
        if detections.detections or changed:
            self._publish(detections.header.stamp)

    def _count_views(self, depth, cam_to_map):
        """Count one view for every track whose point the camera could see in this frame.

        Returns True when a track changed state because of it (a demotion or confirmation).
        """
        changed = False
        if cam_to_map is None or not self.tracker.tracks:
            return changed
        fx, fy, cx, cy = self._k
        for t in self.tracker.tracks:
            p = inverse_transform_point(*cam_to_map, (t.x, t.y, t.z))
            if point_visible(p, depth, fx, fy, cx, cy, self._g('min_range_m'),
                             self._g('max_range_m'), occlusion_tol=self._g('occlusion_tolerance_m')):
                before = t.state
                self.tracker.count_view(t)
                if before != t.state:
                    changed = True
                    self.get_logger().info(
                        f'track {t.id} {t.state} at ({t.x:.2f}, {t.y:.2f}): hit rate '
                        f'{t.hit_rate:.2f} ({t.hits} hits in {t.in_view} views)')
        return changed

    def _publish(self, stamp):
        msg, markers = survivor_messages(self.tracker.tracks, stamp, self._g('map_frame'),
                                         self.tracker.suppressed())
        self._pub.publish(msg)
        self._marker_pub.publish(markers)

    def _log_stats(self):
        s = self.stats
        self.get_logger().info(
            f"pairs {s['pairs']}, detections {s['detections']}, used {s['used']}, low conf"
            f" {s['low_confidence']}, bad depth {s['bad_depth']}, tf failed {s['tf_failed']};"
            f" tracks {len(self.tracker.tracks)}, confirmed {len(self.tracker.confirmed())}")


def main():
    rclpy.init()
    node = SurvivorLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
