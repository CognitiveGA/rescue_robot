import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    node = rclpy.create_node('test_perception_to_triage_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_perception_to_triage(test_node):
    publisher = test_node.create_publisher(String, 'injury_detection', 10)

    # 🕒 Wait until a subscriber connects
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Publish a fake injury detection message
    msg = String()
    fake_injury_data = {
        "image_frame": "fake_frame_data",
        "depth_info": {"depth": 1.2}
    }
    msg.data = json.dumps(fake_injury_data)
    publisher.publish(msg)

    # Allow time for processing
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)