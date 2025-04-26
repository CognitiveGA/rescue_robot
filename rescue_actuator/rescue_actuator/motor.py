"""
Motor module.

This module defines the Motor class, which simulates basic motor behavior
such as moving forward or backward, stopping, and reporting its current status.

Classes:
    Motor: Represents a single motor with movement and status capabilities.

Example:
    >>> left = Motor(name="Left Motor")
    >>> right = Motor(name="Right Motor")
    >>> left.move(30)
    >>> right.move(-20)
    >>> print(left.get_status())
    >>> print(right.get_status())
    >>> left.stop()
    >>> right.stop()
"""

class Motor:
    """
    Represents a motor with basic movement functionality.

    Attributes:
        name (str): The name of the motor.
        max_speed (int): The maximum allowed speed for the motor.
        current_speed (int): The current speed of the motor.
        direction (str): The current direction of movement ("forward", "backward", "stopped").
    """

    def __init__(self, name="Motor", max_speed=100):
        """
        Initialize a Motor instance.

        Args:
            name (str, optional): The name of the motor. Defaults to "Motor".
            max_speed (int, optional): The maximum speed limit for the motor. Defaults to 100.
        """
        self.name = name
        self.max_speed = max_speed
        self.current_speed = 0
        self.direction = "stopped"  # "forward", "backward", "stopped"

    def move(self, speed):
        """
        Move the motor at the specified speed.

        If the speed exceeds the maximum limit, it will be capped at max_speed.

        Args:
            speed (int or float): The desired speed. Positive for forward, negative for backward.
        """
        if speed > self.max_speed:
            speed = self.max_speed
        elif speed < -self.max_speed:
            speed = -self.max_speed

        self.current_speed = speed
        self.direction = "forward" if speed > 0 else ("backward" if speed < 0 else "stopped")

        print(f"[{self.name}] Moving {self.direction} at speed {abs(speed)}")

    def stop(self):
        """
        Stop the motor by setting its speed to zero and updating its direction to 'stopped'.
        """
        self.current_speed = 0
        self.direction = "stopped"
        print(f"[{self.name}] Stopped.")

    def get_status(self):
        """
        Get the current status of the motor.

        Returns:
            dict: A dictionary containing the motor's name, current speed, and direction.
        """
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