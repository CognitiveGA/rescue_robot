import random
import time

class AutonomousNavigation:
    def __init__(self, safety_margin=0.5):
        self.safety_margin = safety_margin  # meters
        self.module_name = "Autonomous Navigation"
        self.navigation_log = []

    def navigate(self, map_data):
        print(f"[{self.module_name}] Planning route using map...")

        quality = map_data.get("quality_score", 0.0)
        if quality < 0.6:
            print("[WARNING] Map quality too low for safe navigation.")
            return {
                "timestamp": time.time(),
                "status": "ABORTED",
                "reason": "Poor map quality"
            }

        # Simulate path planning
        path = [(0, 0), (1, 2), (2, 4), (3, 6)]
        eta = random.randint(3, 8)  # seconds

        navigation_result = {
            "timestamp": time.time(),
            "status": "SUCCESS",
            "path": path,
            "estimated_time": eta
        }

        self.navigation_log.append(navigation_result)
        print(f"[NAVIGATION] Route planned. ETA: {eta}s")
        return navigation_result

    def get_log(self):
        return self.navigation_log


# Example usage
if __name__ == '__main__':
    nav = AutonomousNavigation()
    fake_map = {"quality_score": 0.9}
    outcome = nav.navigate(fake_map)
    print(outcome)
