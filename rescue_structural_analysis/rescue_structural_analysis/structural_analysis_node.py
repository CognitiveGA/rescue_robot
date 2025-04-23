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

        # Subscribe to sensor data stream from perception
        self.subscription = self.create_subscription(
            String,
            'env_detection',
            self.structure_callback,
            10)

    def structure_callback(self, msg):
        try:
            structure_data = json.loads(msg.data)
            risk_report = self.risk_assessor.assess(structure_data)
            damage_report = self.damage_detector.detect(structure_data)
            self.get_logger().info(f"Structural reports generated: RISK={risk_report['risk_level']}, OBSTACLE={damage_report['obstacle_detected']}")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode structural data JSON.")


def main(args=None):
    rclpy.init(args=args)
    node = StructuralAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()