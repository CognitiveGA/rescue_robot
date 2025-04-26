"""
Speaker module.

This module defines the Speaker class, which simulates an audio speaker
that can play back messages at a configurable volume.

Classes:
    Speaker: Simulates a speaker device capable of speaking and adjusting volume.

Example:
    >>> speaker = Speaker()
    >>> speaker.activate()
    >>> speaker.speak("Hello, survivor!")
    >>> speaker.set_volume(7)
    >>> speaker.deactivate()
"""

class Speaker:
    """
    Simulated speaker for playing audio messages.

    Attributes:
        volume_level (int): Volume level (0 for mute, 10 for maximum).
        speaker_name (str): Name of the speaker device.
        active (bool): Whether the speaker is currently active.
    """

    def __init__(self, volume_level=5):
        """
        Initialize the Speaker instance.

        Args:
            volume_level (int, optional): Initial volume level (0–10). Defaults to 5.
        """
        self.volume_level = volume_level
        self.speaker_name = "Default Speaker"
        self.active = False

    def activate(self):
        """
        Activate the speaker.
        """
        self.active = True
        print(f"{self.speaker_name} activated at volume {self.volume_level}.")

    def speak(self, audio):
        """
        Play an audio message through the speaker.

        Args:
            audio (str): The audio message to be spoken.

        Raises:
            RuntimeError: If the speaker is not activated.
        """
        if not self.active:
            raise RuntimeError("Speaker not activated!")
        print(f"Speaking out: '{audio}' at volume {self.volume_level}")

    def set_volume(self, new_volume):
        """
        Set a new volume level for the speaker.

        Args:
            new_volume (int): New volume level (clipped between 0 and 10).
        """
        self.volume_level = max(0, min(new_volume, 10))
        print(f"{self.speaker_name} volume set to {self.volume_level}.")

    def deactivate(self):
        """
        Deactivate the speaker.
        """
        self.active = False
        print(f"{self.speaker_name} deactivated.")


# Example usage
if __name__ == "__main__":
    speaker = Speaker()
    speaker.activate()
    speaker.speak("Hello, survivor!")
    speaker.set_volume(7)
    speaker.deactivate()