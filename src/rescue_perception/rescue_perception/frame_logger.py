"""Save labelled camera frames while the robot explores (dataset collection, not for evaluation).

Needs the simulator started with seg_camera:=true. Every `period_s` of sim time it saves the RGB
frame, a YOLO label file made from the segmentation mask (label 1 = survivor) and one row of
meta.csv. These frames show what the detector sees on the exploration path, which is the
distribution the pipeline runs in, so they are the source of hard negatives.
"""
import csv
import json
import os

import message_filters
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from rescue_perception.dataset import boxes_from_mask, yolo_line


def as_array(msg, channels):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return a[:, :msg.width * channels].reshape(msg.height, msg.width, channels)


class FrameLogger(Node):

    def __init__(self):
        super().__init__('frame_logger')
        self.declare_parameter('out_dir', 'results/dataset_explore')
        self.declare_parameter('tag', 'run')
        self.declare_parameter('period_s', 1.0)
        out, self.tag = self.get_parameter('out_dir').value, self.get_parameter('tag').value
        self.period = float(self.get_parameter('period_s').value)
        os.makedirs(f'{out}/images', exist_ok=True)
        os.makedirs(f'{out}/labels', exist_ok=True)
        self.out = out
        new = not os.path.exists(f'{out}/meta.csv')
        self.meta = open(f'{out}/meta.csv', 'a', newline='')
        self.writer = csv.DictWriter(self.meta, fieldnames=['image', 'tag', 'stamp', 'n_boxes', 'boxes'])
        if new:
            self.writer.writeheader()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        rgb = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos)
        seg = message_filters.Subscriber(self, Image, '/camera/segmentation/labels_map',
                                         qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, seg], 10, 0.03)
        self.sync.registerCallback(self.on_pair)
        self.next_t, self.count = 0.0, 0

    def on_pair(self, rgb, seg):
        t = rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9
        if t < self.next_t:
            return
        self.next_t = t + self.period
        boxes = boxes_from_mask(as_array(seg, 3)[:, :, 0] == 1)
        name = f'{self.tag}_{self.count:05d}'
        PILImage.fromarray(as_array(rgb, 3)).save(f'{self.out}/images/{name}.png')
        with open(f'{self.out}/labels/{name}.txt', 'w') as f:
            f.write(''.join(yolo_line(b, rgb.width, rgb.height) + '\n' for b in boxes))
        self.writer.writerow({'image': name, 'tag': self.tag, 'stamp': round(t, 3),
                              'n_boxes': len(boxes), 'boxes': json.dumps(boxes)})
        self.meta.flush()
        self.count += 1


def main():
    rclpy.init()
    node = FrameLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.meta.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
