import time
import math

class StructuralRiskAssessment:
    def __init__(self, collapse_threshold=0.7):
        self.collapse_threshold = collapse_threshold
        self.history = []
        self.module_name = "Structural Risk Assessment"

    def assess(self, structure_data):
        print(f"[{self.module_name}] Assessing structure for collapse risk...")

        acc = structure_data.get("linear_acceleration", {})
        gyro = structure_data.get("angular_velocity", {})

        # Estimate stress as magnitude of acceleration vector (normalized)
        stress_value = math.sqrt(
            acc.get("x", 0.0)**2 +
            acc.get("y", 0.0)**2 +
            acc.get("z", 0.0)**2
        ) / 9.81  # Normalize to g

        # Estimate angle variation as magnitude of angular velocity (degrees/sec)
        angle_shift = math.sqrt(
            gyro.get("x", 0.0)**2 +
            gyro.get("y", 0.0)**2 +
            gyro.get("z", 0.0)**2
        )

        collapse_risk = 0.5 * stress_value + 0.03 * angle_shift
        collapse_risk = min(collapse_risk, 1.0)

        result = {
            "timestamp": time.time(),
            "stress_value": round(stress_value, 3),
            "angle_variation": round(angle_shift, 3),
            "collapse_risk": round(collapse_risk, 3),
            "risk_level": "HIGH" if collapse_risk >= self.collapse_threshold else "LOW"
        }

        self.history.append(result)
        print(f"[RISK] Collapse risk = {collapse_risk:.2f} → {result['risk_level']}")
        return result
