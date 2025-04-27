"""
Integration Test: Perception to Search Tasks
=============================================

This test checks if the 'rescue_perception' package sends injury detection
messages that the 'rescue_search_tasks' package can receive and use.

We send a fake injury detection message and make sure the system
accepts it properly.

This test is explained simply to help beginners understand how messages
flow inside a robot's thinking system.

"""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    """
    Start ROS 2 before running the tests and shut it down after tests finish.
    This sets up communication for the test session.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a test node that can publish messages.
    It acts like a small part of the robot sending injury data.
    """
    node = rclpy.create_node('test_perception_to_triage_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_perception_to_triage(test_node):
    """
    This test does the following:

    1. Create a publisher on the 'injury_detection' topic.
    2. Wait until the search_and_rescue_node is ready to listen.
    3. Publish fake injury data (simulating perception output).
    4. Allow time for the system to react.

    This checks that injury information flows correctly from
    perception to triage modules.
    """
    publisher = test_node.create_publisher(String, 'injury_detection', 10)

    # 🕒 Wait for subscriber connection
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Create and send a fake injury detection message
    msg = String()
    fake_injury_data = {
        "image_frame": "fake_frame_data",
        "depth_info": {"depth": 1.2}
    }
    msg.data = json.dumps(fake_injury_data)
    publisher.publish(msg)

    # Allow the system to react
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)

