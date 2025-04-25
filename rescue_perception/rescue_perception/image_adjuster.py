class ImageAdjuster:
    def __init__(self, brightness=1.0, contrast=1.0):
        self.brightness = brightness
        self.contrast = contrast

    def adjust(self, image_data):
        # No ROS dependency: adjust pixel array if implemented
        adjusted = {
            'adjusted_data': f"<adjusted_{getattr(image_data, 'data', '<no_data>')}>",
            'brightness': self.brightness,
            'contrast': self.contrast
        }
        return adjusted
