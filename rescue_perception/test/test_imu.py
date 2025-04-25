import time
import pytest
import rclpy
from sensor_msgs.msg import Imu
from rescue_perception.imu import IMU

def test_imu_subscription(node):
    wrapper = IMU(node)
    pub = node.create_publisher(Imu, '/base_imu', 10)

    msg = Imu()
    msg.linear_acceleration.x = 0.1
    msg.angular_velocity.y = 2.5
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

    start = time.time()
    while wrapper.get_data() is None and time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    data = wrapper.get_data()
    assert data is not None
    assert data.linear_acceleration.x == pytest.approx(0.1)
    assert data.angular_velocity.y == pytest.approx(2.5)
