import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rescue_communicator.microphone import Microphone
from rescue_communicator.speaker import Speaker
from rescue_communicator.real_time_reporter import RealTimeReporter
from rescue_communicator.mission_status_notification import MissionStatusNotification

class CommunicatorNode(Node):
    def __init__(self):
        super().__init__('communicator_node')

        # Initialize components
        self.microphone = Microphone()
        self.speaker = Speaker()
        self.reporter = RealTimeReporter()
        self.notifier = MissionStatusNotification()

        # Activate devices (simulate startup)
        self.microphone.activate()
        self.speaker.activate()
        self.reporter.activate()
        self.notifier.activate()

        # Publishers and Subscribers
        self.publisher_audio = self.create_publisher(String, 'audio_in', 10)
        self.subscriber_audio = self.create_subscription(String, 'search_rescue_audio_out', self.receive_audio_callback, 10)

        # Timer to simulate periodic audio capture
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.capture_audio)

    def capture_audio(self):
        # Simulate listening to audio from environment
        audio_data = self.microphone.listen()
        msg = String()
        msg.data = audio_data
        self.publisher_audio.publish(msg)
        self.reporter.report("Microphone captured new audio sample.")

    def receive_audio_callback(self, msg):
        # Receive audio message and send through speaker
        self.get_logger().info(f"Received audio to play: {msg.data}")
        self.speaker.speak(msg.data)
        self.notifier.notify({"message": "Playing received audio", "urgency": 0.3})


def main(args=None):
    rclpy.init(args=args)
    node = CommunicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()