class LiDAR:
    def __init__(self, range_meters=100, scan_rate=10):
        self.range_meters = range_meters  # Maximum detection range
        self.scan_rate = scan_rate        # Scans per second
        self.lidar_name = "Default LiDAR"
        self.connected = False

    def connect(self):
        self.connected = True
        print(f"{self.lidar_name} connected with {self.range_meters}m range at {self.scan_rate} Hz scan rate.")

    def scan(self):
        if not self.connected:
            raise RuntimeError("LiDAR not connected!")
        # Simulate a LiDAR scan returning point cloud data
        print(f"Scanning environment up to {self.range_meters} meters...")
        return {
            "points": [
                (1.0, 2.0, 0.0),
                (3.5, 4.2, 0.0),
                (5.1, -1.2, 0.0)
            ],
            "range": self.range_meters
        }

    def disconnect(self):
        self.connected = False
        print(f"{self.lidar_name} disconnected.")


# Example usage (for quick test)
if __name__ == "__main__":
    lidar = LiDAR()
    lidar.connect()
    points = lidar.scan()
    print(points)
    lidar.disconnect()
