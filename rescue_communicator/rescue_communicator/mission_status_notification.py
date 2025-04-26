"""
Mission Status Notification module.

This module defines the MissionStatusNotification class, which simulates a notifier
for mission status updates based on an urgency threshold.

Classes:
    MissionStatusNotification: Sends mission status notifications based on urgency.

Example:
    >>> notifier = MissionStatusNotification()
    >>> notifier.activate()
    >>> notifier.notify({"message": "Victim detected nearby.", "urgency": 0.8})
    >>> notifier.notify({"message": "Scanning area...", "urgency": 0.4})
    >>> notifier.deactivate()
"""

class MissionStatusNotification:
    """
    Simulated mission status notifier that triggers alerts based on urgency levels.

    Attributes:
        urgency_threshold (float): Threshold (0.0 to 1.0) to classify a message as urgent.
        active (bool): Whether the notifier is currently active.
        notifier_name (str): Name of the notifier device.
    """

    def __init__(self, urgency_threshold=0.7):
        """
        Initialize the MissionStatusNotification instance.

        Args:
            urgency_threshold (float, optional): Minimum urgency level to trigger an alert. Defaults to 0.7.
        """
        self.urgency_threshold = urgency_threshold
        self.active = False
        self.notifier_name = "Default Mission Status Notifier"

    def activate(self):
        """
        Activate the mission status notifier.
        """
        self.active = True
        print(f"{self.notifier_name} activated with urgency threshold {self.urgency_threshold}.")

    def notify(self, status):
        """
        Send a mission status update.

        Args:
            status (dict): A dictionary containing at least:
                - 'message' (str): The status message.
                - 'urgency' (float, optional): Urgency level (defaults to 0.5 if not provided).

        Raises:
            RuntimeError: If the notifier is not activated.
        """
        if not self.active:
            raise RuntimeError("Mission Status Notifier not activated!")
        urgency = status.get("urgency", 0.5)
        if urgency >= self.urgency_threshold:
            print(f"[ALERT] Urgent mission update: {status['message']}")
        else:
            print(f"[INFO] Mission update: {status['message']}")

    def deactivate(self):
        """
        Deactivate the mission status notifier.
        """
        self.active = False
        print(f"{self.notifier_name} deactivated.")


# Example usage
if __name__ == "__main__":
    notifier = MissionStatusNotification()
    notifier.activate()
    notifier.notify({"message": "Victim detected nearby.", "urgency": 0.8})
    notifier.notify({"message": "Scanning area...", "urgency": 0.4})
    notifier.deactivate()