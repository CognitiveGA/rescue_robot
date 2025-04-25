import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    node = rclpy.create_node('test_perception_to_core_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_env_detection_to_navigation(test_node):
    publisher = test_node.create_publisher(String, 'env_detection', 10)

    # 🕒 WAIT until subscriber is ready
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscription connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Now it is safe to publish
    msg = String()
    fake_env_data = {
        "points_frame_id": "lidar_frame",
        "sonar_range": 2.5,
        "point_cloud": [(1, 2, 0), (2, 3, 0)]
    }
    msg.data = json.dumps(fake_env_data)
    publisher.publish(msg)

    # Allow processing time
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)
