import math

# Coordinates of points A, B, and C
x1, y1 = -1.0, 2.0  # Change these values to your actual coordinates for point A
x2, y2 = 0.0, 0.0  # Change these values to your actual coordinates for point B
x3, y3 = 1.0, 2.0  # Change these values to your actual coordinates for point C

# Calculate vectors AB and BC
ABx, ABy = x1 - x2, y1 - y2
BCx, BCy = x3 - x2, y3 - y2

# Calculate the dot product of AB and BC
dot_product = ABx * BCx + ABy * BCy

# Calculate the magnitudes of vectors AB and BC
magnitude_AB = math.sqrt(ABx**2 + ABy**2)
magnitude_BC = math.sqrt(BCx**2 + BCy**2)

# Calculate the cosine of the angle at point B
cosine_theta = dot_product / (magnitude_AB * magnitude_BC)

# Calculate the angle in radians
theta_radians = math.acos(cosine_theta)

# Convert the angle to degrees
theta_degrees = math.degrees(theta_radians)

print("Angle at point B (in degrees):", theta_degrees)
