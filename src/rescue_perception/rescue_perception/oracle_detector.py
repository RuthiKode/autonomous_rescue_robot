"""ORACLE detector: a test double that publishes detections from ground truth.

  *** It uses the true survivor poses. Never report its output as a detection result. ***

Why it exists: the pretrained COCO detector finds none of the low-poly survivors
(docs/PHASE4.md), so until the detector is fine-tuned (plan week 6) nothing would reach
the survivor localizer. This node lets everything downstream of the detector (depth,
TF, tracks, confirmation, exploration, scoring) be exercised on its own.

For every depth frame it projects the primitive shapes of each true survivor into the
camera image, takes their bounding box, and publishes it as a Detection2D with a fixed
confidence. A survivor counts as visible only if it is within range, its box is inside the
image, and the depth image agrees with the survivor's distance at the projected centre
(so walls and furniture in front of it hide it; a survivor fully occluded from the camera
is not reported).
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import yaml

from rescue_perception.geometry import inverse_transform_point, quat_rotate
from rescue_perception.localizer_node import depth_to_array


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def survivor_points(posture, x, y, yaw):
    """Sample points on a survivor's primitive shapes, in the world frame."""
    from rescue_simulation.generate_worlds import _survivor_shapes
    shapes = _survivor_shapes({'posture': posture, 'shirt': (1, 1, 1), 'trousers': (1, 1, 1),
                               'skin': (1, 1, 1)})
    q_model = rpy_to_quat(0.0, 0.0, yaw)
    points = []
    for s in shapes:
        px, py, pz, roll, pitch, pyaw = s['pose']
        q_shape = rpy_to_quat(roll, pitch, pyaw)
        if s['geom'] == 'box':
            half = [0.5 * v for v in s['size']]
        elif s['geom'] == 'cylinder':
            half = [s['size'][0], s['size'][0], 0.5 * s['size'][1]]
        else:
            half = [s['size'][0]] * 3
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    local = quat_rotate(q_shape, (sx * half[0], sy * half[1], sz * half[2]))
                    in_model = (local[0] + px, local[1] + py, local[2] + pz)
                    wx, wy, wz = quat_rotate(q_model, in_model)
                    points.append((wx + x, wy + y, wz))
    return points


class OracleDetector(Node):

    def __init__(self):
        super().__init__('oracle_detector')
        p = self.declare_parameter
        p('truth_file', '')
        p('depth_topic', '/camera/depth')
        p('camera_info_topic', '/camera/camera_info')
        p('detections_topic', '/detections')
        p('map_frame', 'map')
        p('max_range_m', 6.0)
        p('rate_hz', 10.0)
        p('confidence', 0.9)
        p('depth_agreement_m', 0.7)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._g = g
        with open(g('truth_file')) as f:
            truth = yaml.safe_load(f)
        self._survivors = [(s['id'], s['posture'], s['position']['x'], s['position']['y'],
                            s['yaw']) for s in truth['survivors']]
        self._points = {sid: survivor_points(post, x, y, yaw)
                        for sid, post, x, y, yaw in self._survivors}
        self._centroid = {sid: (x, y, 0.5 if post == 'standing' else 0.3)
                          for sid, post, x, y, yaw in self._survivors}
        self._k = None
        self._last = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._pub = self.create_publisher(Detection2DArray, g('detections_topic'), 10)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_info, 1)
        self.create_subscription(Image, g('depth_topic'), self._on_depth,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().warn('ORACLE detector: detections come from ground truth')

    def _on_info(self, msg):
        self._k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def _on_depth(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._k is None or (self._last is not None
                               and stamp - self._last < 0.95 / self._g('rate_hz')):
            return
        self._last = stamp
        try:
            tf = self.tf_buffer.lookup_transform(self._g('map_frame'), msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp),
                                                 Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return
        t, r = tf.transform.translation, tf.transform.rotation
        trans, rot = (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)
        fx, fy, cx, cy, w, h = self._k
        depth = depth_to_array(msg)

        def project(point):
            xc, yc, zc = inverse_transform_point(trans, rot, point)
            if zc <= 0.1:
                return None
            return fx * xc / zc + cx, fy * yc / zc + cy, zc

        out = Detection2DArray()
        out.header = msg.header
        for sid, *_ in self._survivors:
            proj = [project(p) for p in self._points[sid]]
            if any(p is None for p in proj):
                continue                       # part of the body is behind the camera
            us, vs = [p[0] for p in proj], [p[1] for p in proj]
            u0, u1, v0, v1 = max(0.0, min(us)), min(w - 1.0, max(us)), max(0.0, min(vs)), \
                min(h - 1.0, max(vs))
            if u1 - u0 < 4 or v1 - v0 < 4:
                continue
            cproj = project(self._centroid[sid])
            if cproj is None or cproj[2] > self._g('max_range_m'):
                continue
            # occlusion: the depth image must agree with the survivor's distance
            uc, vc = (u0 + u1) / 2, (v0 + v1) / 2
            patch = depth[max(0, int(vc) - 6):int(vc) + 7, max(0, int(uc) - 6):int(uc) + 7]
            patch = patch[np.isfinite(patch)]
            if patch.size == 0 or abs(float(np.median(patch)) - cproj[2]) > \
                    self._g('depth_agreement_m'):
                continue
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y = uc, vc
            d.bbox.size_x, d.bbox.size_y = u1 - u0, v1 - v0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = 'person', float(self._g('confidence'))
            d.results.append(hyp)
            out.detections.append(d)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = OracleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
