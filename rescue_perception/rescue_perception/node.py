import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rescue_perception.rgb_camera import RGBCamera
from rescue_perception.lidar import LiDAR
from rescue_perception.sonar import SONAR
from rescue_perception.gps import GPS
from rescue_perception.imu import IMU
from rescue_perception.image_adjuster import ImageAdjuster

import json

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize sensor objects
        self.rgb_camera = RGBCamera()
        self.lidar = LiDAR()
        self.sonar = SONAR()
        self.gps = GPS()
        self.imu = IMU()
        self.image_adjuster = ImageAdjuster()

        # Connect sensors (simulate hardware setup)
        self.rgb_camera.connect()
        self.lidar.connect()
        self.sonar.connect()
        self.gps.connect()
        self.imu.connect()

        # Create publishers
        self.publisher_envdet = self.create_publisher(String, 'env_detection', 10)
        self.publisher_injdet = self.create_publisher(String, 'injury_detection', 10)
        self.publisher_accgyr = self.create_publisher(String, 'imu_data', 10)
        self.publisher_pos = self.create_publisher(String, 'gps_position', 10)

        # Create timer to publish data periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

    def publish_sensor_data(self):
        # Capture and publish environmental detection (LiDAR + SONAR)
        lidar_data = self.lidar.scan()
        sonar_data = self.sonar.detect()
        env_detection = {
            "lidar": lidar_data,
            "sonar": sonar_data
        }
        msg_env = String()
        msg_env.data = json.dumps(env_detection)
        self.publisher_envdet.publish(msg_env)

        # Capture and publish injury detection (RGB Camera + Image Adjuster)
        camera_frame = self.rgb_camera.capture()
        adjusted_frame = self.image_adjuster.adjust(camera_frame)
        msg_injury = String()
        msg_injury.data = json.dumps(adjusted_frame)
        self.publisher_injdet.publish(msg_injury)

        # Capture and publish IMU data
        imu_data = self.imu.get_acceleration_gyro()
        msg_imu = String()
        msg_imu.data = json.dumps(imu_data)
        self.publisher_accgyr.publish(msg_imu)

        # Capture and publish GPS position
        gps_position = self.gps.get_position()
        msg_gps = String()
        msg_gps.data = json.dumps(gps_position)
        self.publisher_pos.publish(msg_gps)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
