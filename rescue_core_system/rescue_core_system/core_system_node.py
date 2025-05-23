import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_core_system.mapping import Mapping
from rescue_core_system.autonomous_navigation import AutonomousNavigation

class CoreSystemNode(Node):
    def __init__(self):
        super().__init__('core_system_node')

        self.mapper = Mapping()
        self.navigator = AutonomousNavigation()

        # Subscriber for environmental data from perception
        self.subscription = self.create_subscription(
            String,
            'env_detection',
            self.env_callback,
            10)

        # Publisher for navigation status updates
        self.nav_status_pub = self.create_publisher(
            String,
            'navigation_status',
            10)

    def env_callback(self, msg):
        try:
            sensor_data = json.loads(msg.data)
            map_data = self.mapper.build_map(sensor_data)
            nav_result = self.navigator.navigate(map_data)
            self.get_logger().info(f"[Core] Navigation status: {nav_result['status']}, ETA: {nav_result.get('estimated_time', 'N/A')}s")

            # Publish a dummy navigation status message
            nav_status_msg = String()
            nav_status_msg.data = json.dumps({
                "status": nav_result['status'],
                "estimated_time": nav_result.get('estimated_time', -1)
            })
            self.nav_status_pub.publish(nav_status_msg)

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in env_detection message.")

def main(args=None):
    rclpy.init(args=args)
    node = CoreSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
