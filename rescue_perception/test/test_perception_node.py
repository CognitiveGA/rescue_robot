import time
import json
import pytest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Range, PointCloud2, Image, Imu
from nav_msgs.msg import Odometry
from rescue_perception.perception_node import PerceptionNode  # ✅ RIGHT

def test_perception_node_end_to_end(node):
    p_node = PerceptionNode()
    received = {'env': None, 'inj': None, 'imu': None, 'gps': None}

    node.create_subscription(String, 'env_detection', lambda m: received.update(env=m.data), 10)
    node.create_subscription(String, 'injury_detection', lambda m: received.update(inj=m.data), 10)
    node.create_subscription(String, 'imu_data', lambda m: received.update(imu=m.data), 10)
    node.create_subscription(String, 'gps_position', lambda m: received.update(gps=m.data), 10)

    pubs = []
    msg_range = Range()
    msg_range.range = 1.23
    msg_range.header.stamp = node.get_clock().now().to_msg()
    pubs.append((node.create_publisher(Range, '/sonar_base', 10), msg_range))

    msg_scan = PointCloud2()
    msg_scan.header.stamp = node.get_clock().now().to_msg()
    msg_scan.header.frame_id = 'test'
    pubs.append((node.create_publisher(PointCloud2, '/scan_raw', 10), msg_scan))

    msg_img = Image()
    msg_img.data = b''
    msg_img.header.stamp = node.get_clock().now().to_msg()
    pubs.append((node.create_publisher(Image, '/head_front_camera/rgb/image_raw', 10), msg_img))

    msg_imu = Imu()
    msg_imu.linear_acceleration.x = 0.5
    msg_imu.angular_velocity.z = 0.1
    msg_imu.header.stamp = node.get_clock().now().to_msg()
    pubs.append((node.create_publisher(Imu, '/base_imu', 10), msg_imu))

    msg_odom = Odometry()
    msg_odom.pose.pose.position.x = 9.9
    msg_odom.header.stamp = node.get_clock().now().to_msg()
    pubs.append((node.create_publisher(Odometry, '/ground_truth_odom', 10), msg_odom))

    for pub, msg in pubs:
        pub.publish(msg)

    start = time.time()
    while any(v is None for v in received.values()) and time.time() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(p_node, timeout_sec=0.05)

    assert received['env'] is not None
    data_env = json.loads(received['env'])
    assert data_env['sonar_range'] == pytest.approx(1.23)
    assert received['inj'] is not None
    assert received['imu'] is not None
    assert received['gps'] is not None

    p_node.destroy_node()
