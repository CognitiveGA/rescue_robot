"""
Microphone module.

This module defines the Microphone class, which simulates the behavior of a microphone
device with adjustable sensitivity and optional noise cancellation.

Classes:
    Microphone: Represents a microphone device that can capture simulated audio data.

Example:
    >>> mic = Microphone()
    >>> mic.activate()
    >>> audio = mic.listen()
    >>> print(audio)
    >>> mic.deactivate()
"""

class Microphone:
    """
    Simulated microphone device for capturing audio data.

    Attributes:
        sensitivity (float): Sensitivity level of the microphone (0.0 to 1.0).
        noise_cancellation (bool): Whether noise cancellation is enabled.
        microphone_name (str): Name of the microphone device.
        active (bool): Whether the microphone is currently active.
    """

    def __init__(self, sensitivity=0.8, noise_cancellation=True):
        """
        Initialize the Microphone instance.

        Args:
            sensitivity (float, optional): Sensitivity setting of the microphone. Defaults to 0.8.
            noise_cancellation (bool, optional): Enable or disable noise cancellation. Defaults to True.
        """
        self.sensitivity = sensitivity  # 0.0 to 1.0 sensitivity
        self.noise_cancellation = noise_cancellation
        self.microphone_name = "Default Microphone"
        self.active = False

    def activate(self):
        """
        Activate the microphone, allowing it to capture audio.
        """
        self.active = True
        print(f"{self.microphone_name} activated. Sensitivity: {self.sensitivity}, Noise Cancellation: {self.noise_cancellation}")

    def listen(self):
        """
        Capture audio data using the microphone.

        Returns:
            str: Simulated audio data.

        Raises:
            RuntimeError: If the microphone is not activated.
        """
        if not self.active:
            raise RuntimeError("Microphone not activated!")
        # Simulate capturing audio
        print(f"Listening... Capturing audio with sensitivity {self.sensitivity}.")
        return "<simulated_audio_data>"

    def deactivate(self):
        """
        Deactivate the microphone, stopping it from capturing audio.
        """
        self.active = False
        print(f"{self.microphone_name} deactivated.")


# Example usage
if __name__ == "__main__":
    mic = Microphone()
    mic.activate()
    audio = mic.listen()
    print(audio)
    mic.deactivate()