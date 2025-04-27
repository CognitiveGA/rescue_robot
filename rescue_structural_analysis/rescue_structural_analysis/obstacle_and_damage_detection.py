import time
import random

class ObstacleAndDamageDetection:
    def __init__(self, detection_sensitivity=0.5):
        self.detection_sensitivity = detection_sensitivity  # 0.0 to 1.0
        self.module_name = "Obstacle and Damage Detection"
        self.detected_obstacles = []

    def detect(self, structure_data):
        print(f"[{self.module_name}] Scanning structure data for debris and damage...")

        point_cloud_frame = structure_data.get("point_cloud", "unknown_frame")
        infrared_pattern = structure_data.get("infrared_intensity", 0.5)

        # Simulated obstacle chance based on sensor data quality
        if infrared_pattern is None:
            infrared_pattern = 0.0  # Default safe value

        obstacle_chance = 0.2 + infrared_pattern * 0.5

        is_obstacle = obstacle_chance > self.detection_sensitivity

        result = {
            "timestamp": time.time(),
            "obstacle_detected": is_obstacle,
            "obstacle_count_estimate": int(obstacle_chance * 10) if is_obstacle else 0,
            "infrared_intensity": infrared_pattern,
            "sensor_frame": point_cloud_frame
        }

        if is_obstacle:
            self.detected_obstacles.append(result)
            print(f"[DETECTED] Obstacle(s) found: ~{result['obstacle_count_estimate']} object(s)")
        else:
            print("[CLEAR] No major obstacles detected.")

        return result

    def get_obstacles(self):
        return self.detected_obstacles


# Example usage
if __name__ == '__main__':
    detector = ObstacleAndDamageDetection()
    fake_data = {"point_cloud": [(1, 2, 0), (3, 4, 0)], "infrared_intensity": 0.7}
    outcome = detector.detect(fake_data)
    print(outcome)
