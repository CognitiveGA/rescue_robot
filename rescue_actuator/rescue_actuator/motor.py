class Motor:
    def __init__(self, name="Motor", max_speed=100):
        self.name = name
        self.max_speed = max_speed
        self.current_speed = 0
        self.direction = "stopped"  # "forward", "backward", "stopped"

    def move(self, speed):
        if speed > self.max_speed:
            speed = self.max_speed
        elif speed < -self.max_speed:
            speed = -self.max_speed

        self.current_speed = speed
        self.direction = "forward" if speed > 0 else ("backward" if speed < 0 else "stopped")

        print(f"[{self.name}] Moving {self.direction} at speed {abs(speed)}")

    def stop(self):
        self.current_speed = 0
        self.direction = "stopped"
        print(f"[{self.name}] Stopped.")

    def get_status(self):
        return {
            "name": self.name,
            "speed": self.current_speed,
            "direction": self.direction
        }


# Example usage
if __name__ == '__main__':
    left = Motor(name="Left Motor")
    right = Motor(name="Right Motor")
    left.move(30)
    right.move(-20)
    print(left.get_status())
    print(right.get_status())
    left.stop()
    right.stop()
