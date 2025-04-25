import time
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rescue_perception.gps import GPS

def test_gps_subscription(node):
    wrapper = GPS(node)
    pub = node.create_publisher(Odometry, '/ground_truth_odom', 10)

    msg = Odometry()
    msg.pose.pose.position.x = 5.5
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    start = time.time()
    while wrapper.get_position() is None and time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    pos = wrapper.get_position()
    assert pos is not None
    assert pos.pose.pose.position.x == pytest.approx(5.5)
