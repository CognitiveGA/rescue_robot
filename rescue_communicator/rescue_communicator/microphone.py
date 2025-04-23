class Microphone:
    def __init__(self, sensitivity=0.8, noise_cancellation=True):
        self.sensitivity = sensitivity  # 0.0 to 1.0 sensitivity
        self.noise_cancellation = noise_cancellation
        self.microphone_name = "Default Microphone"
        self.active = False

    def activate(self):
        self.active = True
        print(f"{self.microphone_name} activated. Sensitivity: {self.sensitivity}, Noise Cancellation: {self.noise_cancellation}")

    def listen(self):
        if not self.active:
            raise RuntimeError("Microphone not activated!")
        # Simulate capturing audio
        print(f"Listening... Capturing audio with sensitivity {self.sensitivity}.")
        return "<simulated_audio_data>"

    def deactivate(self):
        self.active = False
        print(f"{self.microphone_name} deactivated.")


# Example usage
if __name__ == "__main__":
    mic = Microphone()
    mic.activate()
    audio = mic.listen()
    print(audio)
    mic.deactivate()
