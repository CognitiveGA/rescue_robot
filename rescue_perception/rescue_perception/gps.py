import random

class GPS:
    def __init__(self, update_rate=1.0):
        self.update_rate = update_rate  # Updates per second
        self.gps_name = "Default GPS"
        self.connected = False

    def connect(self):
        self.connected = True
        print(f"{self.gps_name} connected with update rate {self.update_rate} Hz.")

    def get_position(self):
        if not self.connected:
            raise RuntimeError("GPS not connected!")
        # Simulate getting a GPS position (latitude, longitude)
        latitude = random.uniform(-90.0, 90.0)
        longitude = random.uniform(-180.0, 180.0)
        print(f"GPS position acquired: ({latitude:.6f}, {longitude:.6f})")
        return {
            "latitude": latitude,
            "longitude": longitude
        }

    def disconnect(self):
        self.connected = False
        print(f"{self.gps_name} disconnected.")


# Example usage
if __name__ == "__main__":
    gps = GPS()
    gps.connect()
    pos = gps.get_position()
    print(pos)
    gps.disconnect()
