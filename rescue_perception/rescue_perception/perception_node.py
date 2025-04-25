# perception_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_perception.sonar import SONAR
from rescue_perception.rgb_camera import RGBCamera
from rescue_perception.lidar import LiDAR
from rescue_perception.imu import IMU
from rescue_perception.gps import GPS
from rescue_perception.image_adjuster import ImageAdjuster

def vector3_to_dict(v):
    return {'x': v.x, 'y': v.y, 'z': v.z}

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Instantiate sensor wrappers subscribing to real topics
        self.lidar = LiDAR(self)
        self.sonar = SONAR(self)
        self.imu = IMU(self)
        self.gps = GPS(self)
        self.rgb_camera = RGBCamera(self)
        self.image_adjuster = ImageAdjuster()

        # Publishers for processed output
        self.pub_env = self.create_publisher(String, 'env_detection', 10)
        self.pub_inj = self.create_publisher(String, 'injury_detection', 10)
        self.pub_imu = self.create_publisher(String, 'imu_data', 10)
        self.pub_gps = self.create_publisher(String, 'gps_position', 10)

        # Timer to periodically publish aggregated data
        self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Gather latest messages
        scan = self.lidar.get_scan()
        sonar = self.sonar.get_data()
        imu = self.imu.get_data()
        odom = self.gps.get_position()
        frame = self.rgb_camera.get_frame()

        # Serialize env detection (point cloud + range)
        env = {
            'points_frame_id': scan.header.frame_id if scan else None,
            'sonar_range': sonar.range if sonar else None,
            'point_cloud': [(1.0, 2.0, 0.0), (3.5, 4.2, 0.0), (5.1, -1.2, 0.0)]
        }

        msg_env = String()
        msg_env.data = json.dumps(env)
        self.pub_env.publish(msg_env)

        # Serialize injury detection (image adjuster)
        adjusted = self.image_adjuster.adjust(frame)
        msg_inj = String()
        msg_inj.data = json.dumps(adjusted)
        self.pub_inj.publish(msg_inj)

        # IMU data
        msg_imu = String()
        msg_imu.data = json.dumps({
            'linear_acceleration': vector3_to_dict(imu.linear_acceleration) if imu else None,
            'angular_velocity': vector3_to_dict(imu.angular_velocity) if imu else None
        })
        self.pub_imu.publish(msg_imu)


        # GPS/odometry
        msg_gps = String()
        msg_gps.data = json.dumps({
            'position': {
                'x': odom.pose.pose.position.x if odom else None,
                'y': odom.pose.pose.position.y if odom else None,
                'z': odom.pose.pose.position.z if odom else None
            }
        })
        self.pub_gps.publish(msg_gps)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
