import random
import time

class Mapping:
    def __init__(self, resolution=0.05):
        self.resolution = resolution  # meters per cell
        self.generated_maps = []
        self.module_name = "Mapping Module"

    def build_map(self, sensor_data):
        print(f"[{self.module_name}] Building map from sensor data...")

        # Simulate using point cloud or depth input
        num_points = len(sensor_data.get("point_cloud", []))
        map_quality = random.uniform(0.7, 1.0) if num_points > 0 else 0.0

        generated_map = {
            "timestamp": time.time(),
            "resolution": self.resolution,
            "point_count": num_points,
            "quality_score": map_quality,
            "map_data": f"<fake_map_{int(map_quality*100)}%>"
        }

        self.generated_maps.append(generated_map)
        print(f"[MAP] Generated map with {num_points} points and quality score {map_quality:.2f}")
        return generated_map

    def get_all_maps(self):
        return self.generated_maps


# Example usage
if __name__ == '__main__':
    mapper = Mapping()
    fake_data = {"point_cloud": [(1, 2, 0), (2, 3, 0), (3, 4, 0)]}
    result = mapper.build_map(fake_data)
    print(result)
