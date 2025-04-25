import rclpy
from sensor_msgs.msg import Range

class SONAR:
    def __init__(self, node, topic='/sonar_base', qos_profile=10):
        self.node = node
        self.latest = None
        # Subscribe to real sonar Range messages
        self.sub = node.create_subscription(
            Range,
            topic,
            self._callback,
            qos_profile
        )

    def _callback(self, msg: Range):
        self.latest = msg

    def get_data(self) -> Range:
        return self.latest
