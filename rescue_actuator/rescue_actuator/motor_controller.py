"""
Motor Controller module.

This module defines the MotorController class, a singleton responsible for controlling
two motor instances (left and right). It provides methods to move the motors,
stop them, and query their current status.

Classes:
    MotorController: Singleton class to manage motor actions.

Example:
    >>> mc = MotorController()
    >>> mc.move_both(40, -40)
    >>> print(mc.get_status())
    >>> mc.stop_all()
"""

from rescue_actuator.motor import Motor

class MotorController:
    """
    Singleton class to control two motors: left and right.

    Attributes:
        left_motor (Motor): Instance of the left motor.
        right_motor (Motor): Instance of the right motor.
    """
    _instance = None

    def __new__(cls):
        """
        Create a single instance of MotorController if it does not already exist.

        Returns:
            MotorController: The singleton instance of the class.
        """
        if cls._instance is None:
            cls._instance = super(MotorController, cls).__new__(cls)
            cls._instance.left_motor = Motor(name="Left Motor")
            cls._instance.right_motor = Motor(name="Right Motor")
        return cls._instance

    def move_both(self, left_speed, right_speed):
        """
        Move both motors at the specified speeds.

        Args:
            left_speed (int or float): Speed for the left motor.
            right_speed (int or float): Speed for the right motor.
        """
        print("[MotorController] Commanding both motors...")
        self.left_motor.move(left_speed)
        self.right_motor.move(right_speed)

    def stop_all(self):
        """
        Stop both motors.
        """
        print("[MotorController] Stopping all motors...")
        self.left_motor.stop()
        self.right_motor.stop()

    def get_status(self):
        """
        Get the current status of both motors.

        Returns:
            dict: A dictionary containing the statuses of the left and right motors.
        """
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