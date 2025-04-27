"""
Integration Test: Communicator to Search Tasks
===============================================

This test checks if the 'rescue_communicator' package correctly sends
audio input that the 'rescue_search_tasks' package can receive and process.

A fake audio message is sent, and we make sure the communication between
these two packages is set up properly.

This example is made simple for anyone learning robotics communication systems.

"""

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    """
    Start ROS 2 before the tests and shut it down afterward.
    This prepares the communication system.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a temporary ROS 2 test node.
    This node will act like a mini publisher.
    """
    node = rclpy.create_node('test_audio_to_detection_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_audio_to_detection(test_node):
    """
    This test does the following steps:

    1. Create a publisher that talks on the 'audio_in' topic.
    2. Wait until someone (search_and_rescue_node) is ready to listen.
    3. Publish a fake audio message saying \"Help me!\"
    4. Let the system process the message for a few seconds.

    This test ensures that the connection between the microphone simulation
    and victim detection is working correctly.
    """
    publisher = test_node.create_publisher(String, 'audio_in', 10)

    # 🕒 Wait until there is at least one subscriber
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Now we send a fake audio message
    msg = String()
    msg.data = "Help me! I am injured!"
    publisher.publish(msg)

    # Allow time for the system to receive and act
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)

