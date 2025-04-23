import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_actuator.motor_controller import MotorController

class ActuatorNode(Node):
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
        try:
            cmd = json.loads(msg.data)
            left = cmd.get("left", 0)
            right = cmd.get("right", 0)
            self.motor_controller.move_both(left, right)
            self.get_logger().info(f"Motors set: left={left}, right={right}")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid motor command format. Expecting JSON with 'left' and 'right'.")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()