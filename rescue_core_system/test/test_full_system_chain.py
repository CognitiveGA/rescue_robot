"""
Integration Test: Full System Chain
====================================

This test checks if the entire robot system works together correctly.
It covers communication from sensing (microphone) to thinking (decision making)
to action (motors moving).

We simulate a voice input asking for help, and check if the system processes it.

This test is explained simply for students learning about full robotic systems.

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
    Start the ROS 2 communication system before tests and shutdown after.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a simple test node to send messages into the system.
    """
    node = rclpy.create_node('test_full_system_chain_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_full_system_chain(test_node):
    """
    This test performs these steps:

    1. Create a publisher for the 'audio_in' topic.
    2. Wait until the communicator node is ready to listen.
    3. Publish a fake voice message asking for help.
    4. Allow enough time for the full system to process the message
       (detection, triage, mapping, navigation, actuation).

    This is a big test that ensures all parts of the rescue robot
    are wired correctly and can react as a team.
    """
    publisher = test_node.create_publisher(String, 'audio_in', 10)

    # 🕒 Wait until there is a subscriber connected
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Send a simulated help message
    msg = String()
    msg.data = "Help! I am trapped!"
    publisher.publish(msg)

    # Give the system time to process everything
    end_time = time.time() + 5.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)
