class RealTimeReporter:
    def __init__(self, report_frequency=5.0):
        self.report_frequency = report_frequency  # seconds between reports
        self.active = False
        self.reporter_name = "Default Real-Time Reporter"

    def activate(self):
        self.active = True
        print(f"{self.reporter_name} activated. Reporting every {self.report_frequency} seconds.")

    def report(self, status):
        if not self.active:
            raise RuntimeError("Real-Time Reporter not activated!")
        # Simulate sending a status update
        print(f"[REPORT] Status update: {status}")

    def deactivate(self):
        self.active = False
        print(f"{self.reporter_name} deactivated.")


# Example usage
if __name__ == "__main__":
    reporter = RealTimeReporter()
    reporter.activate()
    reporter.report("All systems operational.")
    reporter.deactivate()
