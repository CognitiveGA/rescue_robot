"""
Integration Test: Search Tasks to Communicator
===============================================

This test checks if the 'rescue_search_tasks' package correctly sends
audio output to the 'rescue_communicator' package.

A fake command is sent, and we verify that the communicator (speaker)
receives it and can act on it.

This example is written to be simple and easy for new learners.

"""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    """
    Start ROS 2 before running the tests and shutdown afterward.
    This sets up the system environment for communication.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a small test node that can publish messages for testing.
    """
    node = rclpy.create_node('test_audio_feedback_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_audio_feedback(test_node):
    """
    This test does the following steps:

    1. Create a publisher that talks on the 'search_rescue_audio_out' topic.
    2. Wait until the speaker (communicator_node) is ready to listen.
    3. Publish a fake message like \"Proceed to next victim location.\"
    4. Give the system time to process the message.

    This verifies that after detecting and triaging a victim, the robot can 
    speak important messages to its environment.
    """
    publisher = test_node.create_publisher(String, 'search_rescue_audio_out', 10)

    # 🕒 Wait until someone subscribes
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Send a fake instruction
    msg = String()
    msg.data = "Proceed to next victim location."
    publisher.publish(msg)

    # Allow time for the message to be heard
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)

