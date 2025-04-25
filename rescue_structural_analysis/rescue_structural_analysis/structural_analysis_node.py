import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from rescue_structural_analysis.structural_risk_assessment import StructuralRiskAssessment
from rescue_structural_analysis.obstacle_and_damage_detection import ObstacleAndDamageDetection

class StructuralAnalysisNode(Node):
    def __init__(self):
        super().__init__('structural_analysis_node')

        self.risk_assessor = StructuralRiskAssessment()
        self.damage_detector = ObstacleAndDamageDetection()

        # Subscribe to sensor topics from Perception Node
        self.sub_env = self.create_subscription(String, '/env_detection', self.env_callback, 10)
        self.sub_imu = self.create_subscription(String, '/imu_data', self.imu_callback, 10)

        self.latest_env = None
        self.latest_imu = None

        # Publishers for structural analysis outputs
        self.pub_risk = self.create_publisher(String, '/risk_info', 10)
        self.pub_obs = self.create_publisher(String, '/obstacle_position', 10)

        # Processing timer
        self.create_timer(1.0, self.process_data)

    def env_callback(self, msg: String):
        try:
            self.latest_env = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in /env_detection")

    def imu_callback(self, msg: String):
        try:
            self.latest_imu = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in /imu_data")

    def process_data(self):
        if not self.latest_env or not self.latest_imu:
            return  # Wait for both sensor inputs

        structure_data = {
            "point_cloud": self.latest_env.get("points_frame_id", "unknown_frame"),
            "infrared_intensity": self.latest_env.get("sonar_range", 0.0),
            "linear_acceleration": self.latest_imu.get("linear_acceleration", {}),
            "angular_velocity": self.latest_imu.get("angular_velocity", {})
        }

        # Analyze structural risk and damage
        risk_report = self.risk_assessor.assess(structure_data)
        damage_report = self.damage_detector.detect(structure_data)

        # Publish structural risk info
        msg_risk = String()
        msg_risk.data = json.dumps(risk_report)
        self.pub_risk.publish(msg_risk)

        # Publish obstacle/damage detection result
        msg_obs = String()
        msg_obs.data = json.dumps(damage_report)
        self.pub_obs.publish(msg_obs)

def main(args=None):
    rclpy.init(args=args)
    node = StructuralAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
