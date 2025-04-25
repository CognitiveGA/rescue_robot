import time
import pytest
import rclpy
from sensor_msgs.msg import Image
from rescue_perception.rgb_camera import RGBCamera

def test_rgb_camera_subscription(node):
    wrapper = RGBCamera(node)
    pub = node.create_publisher(Image, '/head_front_camera/rgb/image_raw', 10)

    msg = Image()
    msg.width = 640
    msg.height = 480
    msg.data = bytes([0] * (640*480*3))
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    start = time.time()
    while wrapper.get_frame() is None and time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    assert wrapper.get_frame() is not None
    assert wrapper.get_frame().width == 640
    assert wrapper.get_frame().height == 480
