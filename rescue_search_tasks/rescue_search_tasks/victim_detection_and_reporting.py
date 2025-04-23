import random
import time

class VictimDetectionAndReporting:
    def __init__(self, detection_threshold=0.6):
        self.detection_threshold = detection_threshold  # 0.0 to 1.0 likelihood
        self.reports = []
        self.module_name = "Victim Detection & Reporting"

    def detect_and_report(self, sensor_data, audio_data):
        # Simulate victim detection using sensor and audio data
        print(f"[{self.module_name}] Processing sensor & audio data for victim detection...")

        detection_score = random.uniform(0.0, 1.0)
        result = {
            "timestamp": time.time(),
            "confidence": detection_score,
            "location": sensor_data.get("gps_position", {"lat": 0.0, "lon": 0.0}),
            "source": "audio" if "help" in audio_data.lower() else "visual"
        }

        if detection_score >= self.detection_threshold:
            print(f"[DETECTED] Victim with confidence {detection_score:.2f}")
            self.reports.append(result)
            return result
        else:
            print(f"[INFO] No reliable victim detection (score={detection_score:.2f})")
            return None

    def get_reports(self):
        return self.reports


# Example usage
if __name__ == "__main__":
    vdr = VictimDetectionAndReporting()
    fake_sensor = {"gps_position": {"lat": 45.0, "lon": 7.0}}
    fake_audio = "Help me please!"
    report = vdr.detect_and_report(fake_sensor, fake_audio)
    print("Report:", report)
