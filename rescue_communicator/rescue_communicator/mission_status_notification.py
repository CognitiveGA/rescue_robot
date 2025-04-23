class MissionStatusNotification:
    def __init__(self, urgency_threshold=0.7):
        self.urgency_threshold = urgency_threshold  # 0.0 to 1.0
        self.active = False
        self.notifier_name = "Default Mission Status Notifier"

    def activate(self):
        self.active = True
        print(f"{self.notifier_name} activated with urgency threshold {self.urgency_threshold}.")

    def notify(self, status):
        if not self.active:
            raise RuntimeError("Mission Status Notifier not activated!")
        # Simulate sending a notification
        urgency = status.get("urgency", 0.5)
        if urgency >= self.urgency_threshold:
            print(f"[ALERT] Urgent mission update: {status['message']}")
        else:
            print(f"[INFO] Mission update: {status['message']}")

    def deactivate(self):
        self.active = False
        print(f"{self.notifier_name} deactivated.")


# Example usage
if __name__ == "__main__":
    notifier = MissionStatusNotification()
    notifier.activate()
    notifier.notify({"message": "Victim detected nearby.", "urgency": 0.8})
    notifier.notify({"message": "Scanning area...", "urgency": 0.4})
    notifier.deactivate()
