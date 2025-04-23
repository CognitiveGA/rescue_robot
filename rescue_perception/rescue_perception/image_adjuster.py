class ImageAdjuster:
    def __init__(self, brightness=1.0, contrast=1.0):
        self.brightness = brightness  # Default no change
        self.contrast = contrast      # Default no change
        self.adjuster_name = "Default Image Adjuster"

    def adjust(self, image_data):
        # Simulate adjusting brightness and contrast on image data
        print(f"Adjusting image with brightness {self.brightness} and contrast {self.contrast}.")
        adjusted_image = {
            "adjusted_data": f"<adjusted_{image_data['data']}>",
            "brightness": self.brightness,
            "contrast": self.contrast
        }
        return adjusted_image


# Example usage
if __name__ == "__main__":
    adjuster = ImageAdjuster(brightness=1.2, contrast=0.8)
    fake_image = {"data": "<fake_image_data>"}
    result = adjuster.adjust(fake_image)
    print(result)
