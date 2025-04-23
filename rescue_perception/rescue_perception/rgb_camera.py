class RGBCamera:
    def __init__(self, resolution=(640, 480), frame_rate=30):
        self.resolution = resolution  # (width, height)
        self.frame_rate = frame_rate  # frames per second
        self.camera_name = "Default RGB Camera"
        self.connected = False

    def connect(self):
        # Simulate connecting to camera hardware
        self.connected = True
        print(f"{self.camera_name} connected with resolution {self.resolution} at {self.frame_rate} FPS.")

    def capture(self):
        if not self.connected:
            raise RuntimeError("Camera not connected!")
        # Simulate capturing an image (normally would return actual frame data)
        print(f"Capturing image at {self.resolution}...")
        return {
            "width": self.resolution[0],
            "height": self.resolution[1],
            "data": "<fake_image_data>"
        }

    def disconnect(self):
        self.connected = False
        print(f"{self.camera_name} disconnected.")


# Example usage (to be removed in real node usage):
if __name__ == "__main__":
    cam = RGBCamera()
    cam.connect()
    frame = cam.capture()
    print(frame)
    cam.disconnect()
