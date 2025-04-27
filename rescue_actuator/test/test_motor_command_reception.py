"""
Integration Test: Core System to Actuator
==========================================

This test checks if the 'rescue_core_system' package can send motor control
commands to the 'rescue_actuator' package.

We simulate a motor command and make sure it reaches the motor controller properly.

This example is written simply so that anyone learning robotics can follow it.

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
    Start ROS 2 communication before the tests and shut it down after the tests finish.
    """
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def test_node(rclpy_init_shutdown):
    """
    Create a ROS 2 test node that will publish motor commands.
    """
    node = rclpy.create_node('test_motor_command_node')
    yield node
    node.destroy_node()

@pytest.mark.rostest
def test_motor_command_reception(test_node):
    """
    This test does the following:

    1. Create a publisher on the 'motor_commands' topic.
    2. Wait for the actuator node (motor controller) to subscribe.
    3. Publish a fake motor command (left motor and right motor speeds).
    4. Allow time for the actuator to respond.

    This test ensures that movement commands can flow from
    the brain of the robot to the motors.
    """
    publisher = test_node.create_publisher(String, 'motor_commands', 10)

    # 🕒 Wait until a subscriber connects
    start_time = time.time()
    while publisher.get_subscription_count() == 0:
        if time.time() - start_time > 5.0:
            raise TimeoutError("Timed out waiting for subscriber connection!")
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Create and send a fake motor command
    msg = String()
    motor_cmd = {
        "left": 30,
        "right": -30
    }
    msg.data = json.dumps(motor_cmd)
    publisher.publish(msg)

    # Allow the system to react
    end_time = time.time() + 2.0
    while time.time() < end_time:
        rclpy.spin_once(test_node, timeout_sec=0.1)
