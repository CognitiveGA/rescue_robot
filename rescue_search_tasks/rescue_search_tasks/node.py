import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_search_tasks.victim_detection_and_reporting import VictimDetectionAndReporting
from rescue_search_tasks.triage_system import TriageSystem

class SearchAndRescueNode(Node):
    def __init__(self):
        super().__init__('search_and_rescue_node')

        self.victim_detector = VictimDetectionAndReporting()
        self.triage_system = TriageSystem()

        # Subscriptions
        self.subscription_sensor = self.create_subscription(String, 'injury_detection', self.sensor_callback, 10)
        self.subscription_audio = self.create_subscription(String, 'audio_in', self.audio_callback, 10)

        # Publisher for output audio (e.g., alerts or commands)
        self.audio_out_publisher = self.create_publisher(String, 'search_rescue_audio_out', 10)

        # Internal buffers to simulate multi-sensor processing
        self.latest_sensor_data = None
        self.latest_audio_data = None

    def sensor_callback(self, msg):
        try:
            self.latest_sensor_data = json.loads(msg.data)
            self.process_data()
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode sensor data JSON.")

    def audio_callback(self, msg):
        self.latest_audio_data = msg.data
        self.process_data()

    def process_data(self):
        if self.latest_sensor_data is not None and self.latest_audio_data is not None:
            detection = self.victim_detector.detect_and_report(self.latest_sensor_data, self.latest_audio_data)
            triage = self.triage_system.prioritize_victims(self.latest_sensor_data, self.latest_audio_data)

            if detection:
                response = {
                    "detection": detection,
                    "triage": triage
                }
                msg = String()
                msg.data = json.dumps(response)
                self.audio_out_publisher.publish(msg)
                self.get_logger().info("Published detection + triage info as audio out.")

            # Reset after processing to avoid duplicate processing
            self.latest_sensor_data = None
            self.latest_audio_data = None


def main(args=None):
    rclpy.init(args=args)
    node = SearchAndRescueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
