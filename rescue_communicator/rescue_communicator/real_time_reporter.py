"""
Real-Time Reporter module.

This module defines the RealTimeReporter class, which simulates a device
that sends periodic real-time status updates.

Classes:
    RealTimeReporter: Sends real-time status reports at a configured frequency.

Example:
    >>> reporter = RealTimeReporter()
    >>> reporter.activate()
    >>> reporter.report("All systems operational.")
    >>> reporter.deactivate()
"""

class RealTimeReporter:
    """
    Simulated real-time status reporter for mission or system updates.

    Attributes:
        report_frequency (float): Time interval in seconds between reports.
        active (bool): Whether the reporter is currently active.
        reporter_name (str): Name of the reporter device.
    """

    def __init__(self, report_frequency=5.0):
        """
        Initialize the RealTimeReporter instance.

        Args:
            report_frequency (float, optional): Frequency in seconds between reports. Defaults to 5.0.
        """
        self.report_frequency = report_frequency  # seconds between reports
        self.active = False
        self.reporter_name = "Default Real-Time Reporter"

    def activate(self):
        """
        Activate the real-time reporter.
        """
        self.active = True
        print(f"{self.reporter_name} activated. Reporting every {self.report_frequency} seconds.")

    def report(self, status):
        """
        Send a real-time status update.

        Args:
            status (str): The status message to report.

        Raises:
            RuntimeError: If the reporter is not activated.
        """
        if not self.active:
            raise RuntimeError("Real-Time Reporter not activated!")
        print(f"[REPORT] Status update: {status}")

    def deactivate(self):
        """
        Deactivate the real-time reporter.
        """
        self.active = False
        print(f"{self.reporter_name} deactivated.")


# Example usage
if __name__ == "__main__":
    reporter = RealTimeReporter()
    reporter.activate()
    reporter.report("All systems operational.")
    reporter.deactivate()