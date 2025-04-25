import time
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from rescue_perception.lidar import LiDAR

def test_lidar_subscription(node):
    wrapper = LiDAR(node)
    pub = node.create_publisher(PointCloud2, '/scan_raw', 10)

    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    start = time.time()
    while wrapper.get_scan() is None and time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    assert wrapper.get_scan() is not None
