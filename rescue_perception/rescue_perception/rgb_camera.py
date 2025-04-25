import rclpy
from sensor_msgs.msg import Image

class RGBCamera:
    def __init__(self, node, topic='/head_front_camera/rgb/image_raw', qos_profile=10):
        self.node = node
        self.latest = None
        # Subscribe to camera raw images
        self.sub = node.create_subscription(
            Image,
            topic,
            self._callback,
            qos_profile
        )

    def _callback(self, msg: Image):
        self.latest = msg

    def get_frame(self) -> Image:
        return self.latest
