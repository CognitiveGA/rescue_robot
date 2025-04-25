import unittest
from rescue_structural_analysis.structural_risk_assessment import StructuralRiskAssessment

class TestStructuralRiskAssessment(unittest.TestCase):
    def setUp(self):
        self.assessor = StructuralRiskAssessment()

    def test_low_risk(self):
        # Values intentionally below the threshold
        data = {
            "linear_acceleration": {"x": 0.1, "y": 0.1, "z": 9.7},
            "angular_velocity": {"x": 0.1, "y": 0.1, "z": 0.1}
        }
        result = self.assessor.assess(data)
        self.assertEqual(result["risk_level"], "LOW")
        self.assertLess(result["collapse_risk"], self.assessor.collapse_threshold)

    def test_high_risk(self):
        # Values intentionally well above the threshold
        data = {
            "linear_acceleration": {"x": 5.0, "y": 5.0, "z": 5.0},
            "angular_velocity": {"x": 5.0, "y": 5.0, "z": 5.0}
        }
        result = self.assessor.assess(data)
        self.assertEqual(result["risk_level"], "HIGH")
        self.assertGreaterEqual(result["collapse_risk"], self.assessor.collapse_threshold)

if __name__ == '__main__':
    unittest.main()
