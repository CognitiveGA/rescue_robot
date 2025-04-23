import random
import time

class TriageSystem:
    def __init__(self):
        self.triage_log = []
        self.module_name = "Triage System"

    def prioritize_victims(self, sensor_data, audio_data):
        print(f"[{self.module_name}] Analyzing victim condition and assigning priority...")

        # Simulate severity score
        severity_score = random.randint(1, 5)  # 1 = low, 5 = critical
        reason = "unknown"

        if "pain" in audio_data.lower():
            severity_score += 1
            reason = "verbal distress"
        if sensor_data.get("movement_detected", False):
            severity_score -= 1
            reason = "movement detected"

        severity_score = max(1, min(severity_score, 5))

        record = {
            "timestamp": time.time(),
            "priority": severity_score,
            "reason": reason,
            "audio": audio_data,
            "sensor_snapshot": sensor_data
        }

        self.triage_log.append(record)
        print(f"[TRIAGE] Victim prioritized at level {severity_score}: {reason}")
        return record

    def get_triage_log(self):
        return self.triage_log


# Example usage
if __name__ == "__main__":
    ts = TriageSystem()
    fake_sensor = {"movement_detected": True}
    fake_audio = "I am in pain!"
    triage = ts.prioritize_victims(fake_sensor, fake_audio)
    print("Triage Result:", triage)
