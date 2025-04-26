"""
Communicator Node module.

This module defines the CommunicatorNode class, which manages communication-related devices
(microphone, speaker, real-time reporter, and mission status notifier) in a ROS 2 node.

It publishes and subscribes to audio messages and simulates real-time audio capture and playback.

Classes:
    CommunicatorNode: A ROS 2 node handling audio communication and mission reporting.

Functions:
    main(args=None): Entry point to run the node.

Example:
    To run the node:
    >>> python communicator_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rescue_communicator.microphone import Microphone
from rescue_communicator.speaker import Speaker
from rescue_communicator.real_time_reporter import RealTimeReporter
from rescue_communicator.mission_status_notification import MissionStatusNotification

class CommunicatorNode(Node):
    """
    ROS 2 node for managing communication devices and handling audio data.

    Attributes:
        microphone (Microphone): Simulated microphone device.
        speaker (Speaker): Simulated speaker device.
        reporter (RealTimeReporter): Reports real-time system events.
        notifier (MissionStatusNotification): Sends mission status updates.
        publisher_audio (Publisher): Publishes captured audio data to the 'audio_in' topic.
        subscriber_audio (Subscription): Subscribes to audio messages from the 'search_rescue_audio_out' topic.
        timer (Timer): Periodic timer to capture audio at a fixed rate.
    """

    def __init__(self):
        """
        Initialize the CommunicatorNode, its devices, publishers, subscribers, and timers.
        """
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
        self.subscriber_audio = self.create_subscription(
            String, 'search_rescue_audio_out', self.receive_audio_callback, 10)

        # Timer to simulate periodic audio capture
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.capture_audio)

    def capture_audio(self):
        """
        Simulate capturing audio from the environment and publish it to the 'audio_in' topic.
        """
        audio_data = self.microphone.listen()
        msg = String()
        msg.data = audio_data
        self.publisher_audio.publish(msg)
        self.reporter.report("Microphone captured new audio sample.")

    def receive_audio_callback(self, msg):
        """
        Callback function to handle received audio messages.

        Args:
            msg (std_msgs.msg.String): Incoming audio message to be played through the speaker.
        """
        self.get_logger().info(f"Received audio to play: {msg.data}")
        self.speaker.speak(msg.data)
        self.notifier.notify({"message": "Playing received audio", "urgency": 0.3})


def main(args=None):
    """
    Entry point for the CommunicatorNode.

    Args:
        args (list, optional): Command-line arguments for ROS 2 initialization.
    """
    rclpy.init(args=args)
    node = CommunicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()