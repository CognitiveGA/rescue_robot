import rclpy
from sensor_msgs.msg import PointCloud2

class LiDAR:
    def __init__(self, node, topic='/scan_raw', qos_profile=10):
        self.node = node
        self.latest = None
        # Subscribe to point cloud scans
        self.sub = node.create_subscription(
            PointCloud2,
            topic,
            self._callback,
            qos_profile
        )

    def _callback(self, msg: PointCloud2):
        self.latest = msg

    def get_scan(self) -> PointCloud2:
        return self.latest
