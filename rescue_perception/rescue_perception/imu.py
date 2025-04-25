# imu.py
import rclpy
from sensor_msgs.msg import Imu

class IMU:
    def __init__(self, node, topic='/base_imu', qos_profile=10):
        self.node = node
        self.latest = None
        # Subscribe to IMU data
        self.sub = node.create_subscription(
            Imu,
            topic,
            self._callback,
            qos_profile
        )

    def _callback(self, msg: Imu):
        self.latest = msg

    def get_data(self) -> Imu:
        return self.latest
