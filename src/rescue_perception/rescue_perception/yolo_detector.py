"""YOLO person detector: /camera/image_raw to /detections.

Runs in the YOLO virtual environment (it needs PyTorch and Ultralytics), which is created
with --system-site-packages so rclpy and the message packages are importable; see
docs/PHASE4.md. Started by launch/perception.launch.py:

  ~/venvs/yolo/bin/python -m rescue_perception.yolo_detector --ros-args --params-file ...

Pretrained COCO weights, class `person` only: this is the week-5 thin slice that proves
the pipeline end to end. The Ultralytics package is AGPL-3.0 (docs/PLAN.md, stage 4).
Model weights are downloaded by Ultralytics on first use and are not committed.
"""
import os
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def image_to_array(msg):
    """sensor_msgs/Image rgb8 or bgr8 to an (h, w, 3) uint8 RGB array (no cv_bridge)."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    arr = arr.reshape(msg.height, msg.width, 3)
    return arr[:, :, ::-1] if msg.encoding == 'bgr8' else arr


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        p = self.declare_parameter
        p('image_topic', '/camera/image_raw')
        p('detections_topic', '/detections')
        p('model', 'yolov8n.pt')
        p('confidence', 0.25)
        p('iou', 0.5)
        p('image_size', 640)
        p('classes', [0])
        p('device', 'auto')
        p('max_rate_hz', 15.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._g = g

        import torch
        from ultralytics import YOLO
        device = g('device')
        if device == 'auto':
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self._device = device
        self._model = YOLO(os.path.expanduser(g('model')))
        self._names = self._model.names
        # warm up so the first real frame does not pay for CUDA initialisation
        self._model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=g('image_size'),
                            device=device, verbose=False)
        self.get_logger().info(f"model {g('model')} on {device}, input size {g('image_size')},"
                               f" classes {list(g('classes'))}")

        self._pub = self.create_publisher(Detection2DArray, g('detections_topic'), 10)
        self.create_subscription(Image, g('image_topic'), self._on_image,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self._last_stamp = None
        self._times, self._frames, self._skipped = [], 0, 0
        self.create_timer(10.0, self._log_stats)

    def _on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        min_period = 1.0 / float(self._g('max_rate_hz'))
        if self._last_stamp is not None and stamp - self._last_stamp < min_period * 0.95:
            self._skipped += 1
            return
        self._last_stamp = stamp
        img = image_to_array(msg)
        t0 = time.perf_counter()
        result = self._model.predict(
            img, imgsz=self._g('image_size'), conf=self._g('confidence'), iou=self._g('iou'),
            classes=list(self._g('classes')), device=self._device, verbose=False)[0]
        self._times.append(time.perf_counter() - t0)
        self._frames += 1

        out = Detection2DArray()
        out.header = msg.header
        for (x1, y1, x2, y2), conf, cls in zip(result.boxes.xyxy.cpu().numpy(),
                                               result.boxes.conf.cpu().numpy(),
                                               result.boxes.cls.cpu().numpy()):
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x, d.bbox.size_y = float(x2 - x1), float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(self._names[int(cls)])
            hyp.hypothesis.score = float(conf)
            d.results.append(hyp)
            out.detections.append(d)
        self._pub.publish(out)

    def _log_stats(self):
        if self._times:
            t = np.array(self._times)
            self.get_logger().info(
                f'{self._frames} frames, inference mean {1000 * t.mean():.1f} ms,'
                f' max {1000 * t.max():.1f} ms, skipped {self._skipped} by the rate cap')
            self._times.clear()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
