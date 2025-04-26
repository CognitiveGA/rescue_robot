"""
Actuator Node for controlling the motors of a rescue robot.

This node subscribes to the 'motor_commands' topic and commands the motor controller
based on JSON-encoded messages containing left and right motor speeds.

Modules:
    - rclpy: ROS 2 Python client library.
    - std_msgs.msg: Standard message definitions, using String messages.
    - json: For parsing command data.
    - MotorController: Class to control motor movements.

Classes:
    - ActuatorNode: ROS 2 node that listens to motor commands and controls the motors.

Functions:
    - main(): Initializes and spins the ActuatorNode.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_actuator.motor_controller import MotorController

class ActuatorNode(Node):
    """
    Node that subscribes to 'motor_commands' topic and controls the motors using MotorController.
    """
    def __init__(self):
        super().__init__('actuator_node')
        self.motor_controller = MotorController()

        # Subscribe to command topic for motor control
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10)

    def command_callback(self, msg):
        """
        Callback function that processes motor command messages.

        Args:
            msg (String): A JSON-formatted string containing 'left' and 'right' motor values.
        """
        try:
            cmd = json.loads(msg.data)
            left = cmd.get("left", 0)
            right = cmd.get("right", 0)
            self.motor_controller.move_both(left, right)
            self.get_logger().info(f"Motors set: left={left}, right={right}")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid motor command format. Expecting JSON with 'left' and 'right'.")


def main(args=None):
    """
    Entry point for the actuator node.

    Initializes the ROS 2 system, creates the ActuatorNode, and keeps it spinning.
    """
    rclpy.init(args=args)
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()