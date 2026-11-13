"""Fill in the covariances that Gazebo leaves at zero.

  /gz/imu      -> /imu        (sensor_msgs/Imu)
  /gz/odom_raw -> /odom_raw   (nav_msgs/Odometry)

Only the covariance fields change. Values come from config/sensor_covariances.yaml.
"""
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
import yaml

UNUSED = 1.0e6  # variance of state the planar robot does not have (z, roll, pitch)


def imu_covariances(cfg):
    """Return (orientation, angular_velocity, linear_acceleration) 3x3 row-major lists."""
    def diag(v):
        return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]

    orientation = diag(0.0)
    if not cfg.get('orientation_available', False):
        orientation[0] = -1.0
    return (orientation, diag(float(cfg['angular_velocity_variance'])),
            diag(float(cfg['linear_acceleration_variance'])))


def odom_covariances(cfg):
    """Return (pose, twist) 6x6 row-major lists. Order: x y z roll pitch yaw."""
    def diag6(values):
        m = [0.0] * 36
        for i, v in enumerate(values):
            m[i * 7] = float(v)
        return m

    pose = diag6([cfg['pose_position_variance'], cfg['pose_position_variance'], UNUSED,
                  UNUSED, UNUSED, cfg['pose_yaw_variance']])
    twist = diag6([cfg['vx_variance'], cfg['vy_variance'], UNUSED,
                   UNUSED, UNUSED, cfg['vyaw_variance']])
    return pose, twist


class CovarianceStamper(Node):

    def __init__(self):
        super().__init__('covariance_stamper')
        self.declare_parameter('config_file', '')
        with open(self.get_parameter('config_file').value) as f:
            cfg = yaml.safe_load(f)
        self._imu_cov = imu_covariances(cfg['imu'])
        self._odom_cov = odom_covariances(cfg['odom'])

        qos = QoSProfile(depth=50)
        self._imu_pub = self.create_publisher(Imu, '/imu', qos)
        self._odom_pub = self.create_publisher(Odometry, '/odom_raw', qos)
        self.create_subscription(Imu, '/gz/imu', self._on_imu, qos)
        self.create_subscription(Odometry, '/gz/odom_raw', self._on_odom, qos)

    def _on_imu(self, msg):
        (msg.orientation_covariance, msg.angular_velocity_covariance,
         msg.linear_acceleration_covariance) = self._imu_cov
        self._imu_pub.publish(msg)

    def _on_odom(self, msg):
        msg.pose.covariance, msg.twist.covariance = self._odom_cov
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = CovarianceStamper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
