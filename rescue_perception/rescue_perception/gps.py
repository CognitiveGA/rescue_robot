import rclpy
from nav_msgs.msg import Odometry

class GPS:
    def __init__(self, node, topic='/ground_truth_odom', qos_profile=10):
        self.node = node
        self.latest = None
        # Subscribe to odometry as a stand-in for GPS
        self.sub = node.create_subscription(
            Odometry,
            topic,
            self._callback,
            qos_profile
        )

    def _callback(self, msg: Odometry):
        self.latest = msg

    def get_position(self) -> Odometry:
        return self.latest
