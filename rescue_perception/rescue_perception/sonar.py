class SONAR:
    def __init__(self, max_depth=50):
        self.max_depth = max_depth  # Maximum depth detection in meters
        self.sonar_name = "Default SONAR"
        self.connected = False

    def connect(self):
        self.connected = True
        print(f"{self.sonar_name} connected with max depth {self.max_depth} meters.")

    def detect(self):
        if not self.connected:
            raise RuntimeError("SONAR not connected!")
        # Simulate a SONAR depth measurement
        print(f"Detecting depth up to {self.max_depth} meters...")
        return {
            "depth": 25.3  # Simulated constant reading for now
        }

    def disconnect(self):
        self.connected = False
        print(f"{self.sonar_name} disconnected.")


# Example usage
if __name__ == "__main__":
    sonar = SONAR()
    sonar.connect()
    depth = sonar.detect()
    print(depth)
    sonar.disconnect()
