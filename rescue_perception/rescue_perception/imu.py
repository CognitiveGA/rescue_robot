import random

class IMU:
    def __init__(self, update_rate=50):
        self.update_rate = update_rate  # Updates per second
        self.imu_name = "Default IMU"
        self.connected = False

    def connect(self):
        self.connected = True
        print(f"{self.imu_name} connected with update rate {self.update_rate} Hz.")

    def get_acceleration_gyro(self):
        if not self.connected:
            raise RuntimeError("IMU not connected!")
        # Simulate IMU linear acceleration and angular velocity data
        linear_acceleration = (
            random.uniform(-1.0, 1.0),
            random.uniform(-1.0, 1.0),
            random.uniform(-1.0, 1.0)
        )
        angular_velocity = (
            random.uniform(-180.0, 180.0),
            random.uniform(-180.0, 180.0),
            random.uniform(-180.0, 180.0)
        )
        print(f"IMU data: Acceleration {linear_acceleration}, Gyro {angular_velocity}")
        return {
            "linear_acceleration": linear_acceleration,
            "angular_velocity": angular_velocity
        }

    def disconnect(self):
        self.connected = False
        print(f"{self.imu_name} disconnected.")


# Example usage
if __name__ == "__main__":
    imu = IMU()
    imu.connect()
    imu_data = imu.get_acceleration_gyro()
    print(imu_data)
    imu.disconnect()
