from PIL import Image
import numpy as np

directory = "src/maps/cave_01/"
# Load the PGM file
pgm_file = directory+"map.pgm"
pgm_image = Image.open(pgm_file)

# Convert the PGM image to a NumPy array
pgm_data = np.array(pgm_image)

# Create a heightmap image with the same dimensions
heightmap_image = Image.new("L", pgm_image.size)

# Normalize the PGM data and set it as the heightmap
normalized_data = (pgm_data - pgm_data.min()) / (pgm_data.max() - pgm_data.min()) * 255
heightmap_image.putdata(normalized_data.astype(np.uint8).ravel())

# Save the heightmap image
heightmap_image.save(directory+"heightmap.png")

print("Heightmap generated and saved as 'heightmap.png'")
