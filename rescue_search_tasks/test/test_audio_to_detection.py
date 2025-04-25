import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    node = rclpy.create_node('test_audio_to_detection_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_audio_to_detection(test_node):
    publisher = test_node.create_publisher(String, 'audio_in', 10)

    # 🕒 Wait until a subscriber connects
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Publish a fake audio message
    msg = String()
    msg.data = "Help me! I am injured!"
    publisher.publish(msg)

    # Allow time for processing
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)
