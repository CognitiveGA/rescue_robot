import unittest
from rescue_structural_analysis.obstacle_and_damage_detection import ObstacleAndDamageDetection

class TestObstacleAndDamageDetection(unittest.TestCase):
    def setUp(self):
        self.detector = ObstacleAndDamageDetection(detection_sensitivity=0.4)

    def test_obstacle_detected(self):
        data = {
            "point_cloud": "lidar_frame_123",
            "infrared_intensity": 0.9
        }
        result = self.detector.detect(data)
        self.assertTrue(result["obstacle_detected"])
        self.assertGreater(result["obstacle_count_estimate"], 0)

    def test_no_obstacle(self):
        detector = ObstacleAndDamageDetection(detection_sensitivity=0.9)
        data = {
            "point_cloud": "lidar_frame_123",
            "infrared_intensity": 0.1
        }
        result = detector.detect(data)
        self.assertFalse(result["obstacle_detected"])
        self.assertEqual(result["obstacle_count_estimate"], 0)

if __name__ == '__main__':
    unittest.main()
