
# Read the .map file
# Replace 'your_map_file.map' with the actual file path
with open('map.map', 'r') as file:
    map_data = file.read().splitlines()


# Initialize a list to store line coordinates
rectangles = []

# Flag to indicate whether we are currently reading lines
reading_lines = False

# Iterate through the lines in the map data
for line in map_data:
    # Check if we've reached the "LINES" section
    if line.strip() == "LINES":
        reading_lines = True
        continue  # Skip the "LINES" line itself

    # Check if we've reached the end of the lines section
    if reading_lines and line.strip() == "DATA":
        break

    # If we are in the lines section, parse the line coordinates
    if reading_lines:
        line_parts = line.split()
        if len(line_parts) == 4:
            x1, y1, x2, y2 = map(int, line_parts)
            rectangles.append((x1, y1, x2, y2))

from PIL import Image, ImageDraw

# Define the image size
image_width = 800
image_height = 600

# Create a white background image
img = Image.new("RGB", (image_width, image_height), "white")
draw = ImageDraw.Draw(img)

# Define the fill color
fill_color = (0, 0, 0)  
rec= {}

# Draw rectangles and fill them
for i in range(0, len(rectangles), 4):
    chave = i // 4  # Crie uma chave com um nome único
    sub_array = rectangles[i:i+4]  # Crie o subarray
    rec[chave] = sub_array 
    
for chave in rec:
    sub_array = rec[chave]
    x1,y1,x2,y2 = sub_array[0]
    x3,y3,x4,y4 = sub_array[2]
    draw.rectangle([(x1, y1), (x3, y3)], fill="black")

# Save the image as a PNG file
img.save("output.png")

# Show the image (optional)
img.show()
