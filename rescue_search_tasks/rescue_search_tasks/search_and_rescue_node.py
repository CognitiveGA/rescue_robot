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

        # Publisher
        self.audio_out_publisher = self.create_publisher(String, 'search_rescue_audio_out', 10)

        # Internal buffers
        self.latest_sensor_data = None
        self.latest_audio_data = None

    def sensor_callback(self, msg):
        try:
            self.latest_sensor_data = json.loads(msg.data)
            self.get_logger().info("[SENSE] Received sensor data")
            self.process_data()
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode sensor data JSON.")

    def audio_callback(self, msg):
        self.latest_audio_data = msg.data
        self.get_logger().info("[SENSE] Received audio input")
        self.process_data()

    def process_data(self):
        if self.latest_sensor_data is not None and self.latest_audio_data is not None:
            detection = self.detect_victim()
            triage = self.triage_victim()

            if detection:
                self.publish_response(detection, triage)

            # Reset buffers
            self.latest_sensor_data = None
            self.latest_audio_data = None

    def detect_victim(self):
        self.get_logger().info("[PERCEIVE] Running victim detection")
        return self.victim_detector.detect_and_report(self.latest_sensor_data, self.latest_audio_data)

    def triage_victim(self):
        self.get_logger().info("[COGNITION] Running triage assessment")
        return self.triage_system.prioritize_victims(self.latest_sensor_data, self.latest_audio_data)

    def publish_response(self, detection, triage):
        msg = String()
        msg.data = json.dumps({
            "detection": detection,
            "triage": triage
        })
        self.audio_out_publisher.publish(msg)
        self.get_logger().info("[ACT] Published detection and triage info")


def main(args=None):
    rclpy.init(args=args)
    node = SearchAndRescueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
