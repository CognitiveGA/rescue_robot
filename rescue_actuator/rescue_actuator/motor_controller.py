from rescue_actuator.motor import Motor

class MotorController:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(MotorController, cls).__new__(cls)
            cls._instance.left_motor = Motor(name="Left Motor")
            cls._instance.right_motor = Motor(name="Right Motor")
        return cls._instance

    def move_both(self, left_speed, right_speed):
        print("[MotorController] Commanding both motors...")
        self.left_motor.move(left_speed)
        self.right_motor.move(right_speed)

    def stop_all(self):
        print("[MotorController] Stopping all motors...")
        self.left_motor.stop()
        self.right_motor.stop()

    def get_status(self):
        return {
            "left": self.left_motor.get_status(),
            "right": self.right_motor.get_status()
        }


# Example usage
if __name__ == '__main__':
    mc = MotorController()
    mc.move_both(40, -40)
    print(mc.get_status())
    mc.stop_all()
