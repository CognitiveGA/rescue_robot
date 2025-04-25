import time
import pytest
import rclpy
from sensor_msgs.msg import Range
from rescue_perception.sonar import SONAR

def test_sonar_subscription(node):
    wrapper = SONAR(node)
    pub = node.create_publisher(Range, '/sonar_base', 10)

    msg = Range()
    msg.range = 12.34
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    start = time.time()
    while wrapper.get_data() is None and time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    assert wrapper.get_data() is not None
    assert wrapper.get_data().range == pytest.approx(12.34)
