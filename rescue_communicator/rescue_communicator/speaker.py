class Speaker:
    def __init__(self, volume_level=5):
        self.volume_level = volume_level  # Volume level 0 (mute) to 10 (max)
        self.speaker_name = "Default Speaker"
        self.active = False

    def activate(self):
        self.active = True
        print(f"{self.speaker_name} activated at volume {self.volume_level}.")

    def speak(self, audio):
        if not self.active:
            raise RuntimeError("Speaker not activated!")
        # Simulate speaking out loud the received audio
        print(f"Speaking out: '{audio}' at volume {self.volume_level}")

    def set_volume(self, new_volume):
        self.volume_level = max(0, min(new_volume, 10))
        print(f"{self.speaker_name} volume set to {self.volume_level}.")

    def deactivate(self):
        self.active = False
        print(f"{self.speaker_name} deactivated.")


# Example usage
if __name__ == "__main__":
    speaker = Speaker()
    speaker.activate()
    speaker.speak("Hello, survivor!")
    speaker.set_volume(7)
    speaker.deactivate()
