import os
import noise
import random
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import cv2

class HeightMapGenerator:

    def __init__(self, size_x: int, size_y: int, meters_per_pixel: float):
        self.meters_per_pixel = meters_per_pixel
        self.width = self._compute_dimension(int(size_x) )
        self.height = self._compute_dimension(int(size_y))
        # Random offsets
        self.x_offset = random.randint(0, 5000)
        self.y_offset = random.randint(0, 5000)


    def generate_random_heightmap(self):
        shape = (self.height, self.width)
        world = np.zeros(shape, dtype=np.float32)

        scale = 100  # Control the scale of noise patterns
        for i in range(shape[0]):
            for j in range(shape[1]):
                world[i][j] = self.generate_value(i/scale + self.x_offset, j/scale + self.y_offset)

        # Normalize to 0-255 range
        world = np.interp(world, (world.min(), world.max()), (0, 255)).astype(np.uint8)
        # Apply Gaussian blur
        world_blurred = cv2.GaussianBlur(world, (5, 5), 0)
        return world_blurred

    def generate_value(self, x, y):
        # This function combines multiple octaves of Perlin noise
        octaves = 6
        persistence = 0.5
        value = 0
        amplitude = 1.0
        frequency = 1.0
        maxValue = 0  # Used for normalization

        for o in range(octaves):
            value += noise.pnoise2(x * frequency, y * frequency) * amplitude
            maxValue += amplitude
            amplitude *= persistence
            frequency *= 2
        return value / maxValue
    
    def save_to_image(self, data, filename):
        plt.imsave(filename, data, cmap='gray')

    def save_to_image(self, data, filename):
        # Convert the numpy array to an image
        image = Image.fromarray(data, mode='L')  # 'L' mode is for grayscale
        
        # Annotate image with its size in meters
        # draw = ImageDraw.Draw(image)
        # font = ImageFont.truetype("arial.ttf", size=12)  # Ensure you have the 'arial.ttf' font in your directory
        # dimensions = f"{self.width * self.meters_per_pixel}m x {self.height * self.meters_per_pixel}m"
        # draw.text((10, 10), dimensions, font=font, fill=255)

        image.save(filename)
        
    def _compute_dimension(self, meters):
        # Heightmap image size must be square, with a size of 2^n+1
        n = int(np.log2((meters / self.meters_per_pixel) - 1))
        return 2**n + 1

    def generate_and_save(self, filename):
        heightmap_data = self.generate_random_heightmap()
        self.save_to_image(heightmap_data, filename)

    def show_image(self, data):
        plt.figure(figsize=(10, 6))
        plt.imshow(data, cmap='gray')
        dimensions = f"{self.width }m x {self.height}m"
        plt.title(dimensions)
        plt.axis('off')  # Hide axes
        plt.show()

if __name__ == "__main__":
    script_directory = os.path.dirname(os.path.abspath(__file__))
    
    generator = HeightMapGenerator(100, 100, 0.05)  # Assuming each pixel corresponds to 0.1 meters
    heightmap_data = generator.generate_random_heightmap()
    
    height_map_path = script_directory + "/../sim/models/custom_heightmap/materials/textures/heightmap.png"
    generator.save_to_image(heightmap_data, height_map_path)
    generator.show_image(heightmap_data)
