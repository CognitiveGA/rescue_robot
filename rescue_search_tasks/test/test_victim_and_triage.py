import unittest
from unittest.mock import patch
from rescue_search_tasks.victim_detection_and_reporting import VictimDetectionAndReporting
from rescue_search_tasks.triage_system import TriageSystem

class TestVictimDetectionAndReporting(unittest.TestCase):
    def setUp(self):
        self.detector = VictimDetectionAndReporting(detection_threshold=0.5)

    @patch('random.uniform', return_value=0.8)
    def test_detect_victim_high_confidence(self, mock_random):
        sensor_data = {"gps_position": {"lat": 10.0, "lon": 20.0}}
        audio_data = "Help! Over here!"
        result = self.detector.detect_and_report(sensor_data, audio_data)

        self.assertIsNotNone(result)
        self.assertGreaterEqual(result["confidence"], 0.5)
        self.assertEqual(result["source"], "audio")
        self.assertEqual(result["location"], sensor_data["gps_position"])

    @patch('random.uniform', return_value=0.3)
    def test_detect_victim_low_confidence(self, mock_random):
        sensor_data = {}
        audio_data = "All quiet."
        result = self.detector.detect_and_report(sensor_data, audio_data)

        self.assertIsNone(result)


class TestTriageSystem(unittest.TestCase):
    def setUp(self):
        self.triage = TriageSystem()

    @patch('random.randint', return_value=2)
    def test_prioritize_movement_detected(self, mock_random):
        sensor_data = {"movement_detected": True}
        audio_data = "No speech."
        result = self.triage.prioritize_victims(sensor_data, audio_data)

        self.assertIn("priority", result)
        self.assertLessEqual(result["priority"], 5)
        self.assertEqual(result["reason"], "movement detected")

    @patch('random.randint', return_value=3)
    def test_prioritize_pain_reported(self, mock_random):
        sensor_data = {}
        audio_data = "I am in pain!"
        result = self.triage.prioritize_victims(sensor_data, audio_data)

        self.assertGreaterEqual(result["priority"], 1)
        self.assertLessEqual(result["priority"], 5)
        self.assertEqual(result["reason"], "verbal distress")


if __name__ == '__main__':
    unittest.main()
