"""
Integration Test: Perception to Core System
============================================

This test checks if the 'rescue_perception' package correctly publishes
environment detection messages to the 'rescue_core_system' package.

A fake environment detection message is sent, and we verify that the 
communication channel is ready and working.

This test is written in a simple way so that even beginners and students
can understand the flow of messages between two parts of a robot system.

"""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    """
    Start ROS 2 before tests and shut it down after all tests are done.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a temporary ROS 2 node just for testing purposes.
    """
    node = rclpy.create_node('test_perception_to_core_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_env_detection_to_navigation(test_node):
    """
    This test does the following:
    
    1. Create a publisher on the 'env_detection' topic.
    2. Wait until the publisher finds a subscriber (core system listening).
    3. Publish a fake environment detection message.
    4. Give the system a few seconds to process the message.

    No assert on output yet; we ensure the system can talk without crashing.
    """
    publisher = test_node.create_publisher(String, 'env_detection', 10)

    # 🕒 Wait until there is a subscriber ready
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscription connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Now we send a fake message
    msg = String()
    fake_env_data = {
        "points_frame_id": "lidar_frame",
        "sonar_range": 2.5,
        "point_cloud": [(1, 2, 0), (2, 3, 0)]
    }
    msg.data = json.dumps(fake_env_data)
    publisher.publish(msg)

    # Allow some time for the system to react
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)

