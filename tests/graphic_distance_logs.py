import os
import matplotlib.pyplot as plt
import glob

# Define a function to extract the 'distance' value from a file
def extract_distance_from_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.startswith("distance:"):
                return float(line.split(":")[1].strip())
    return None

# Directory paths for the two sets of files
directory_path_1 = "tests/acs_logs"
directory_path_2 = "tests/acs_logs_2"

# Use glob to filter files based on a pattern (e.g., all files with '.log' extension)
file_pattern_1 = os.path.join(directory_path_1, "*.log")
file_pattern_2 = os.path.join(directory_path_2, "*.log")
file_names_1 = glob.glob(file_pattern_1)
file_names_2 = glob.glob(file_pattern_2)

# Extract the 'distance' values from each set of files
distances_1 = [extract_distance_from_file(filename) for filename in file_names_1]
distances_2 = [extract_distance_from_file(filename) for filename in file_names_2]

# Create a list of x-values (0, 1, 2, ...) for the number of files in each set
x_values_1 = list(range(len(file_names_1)))
x_values_2 = list(range(len(file_names_2)))

# Create a line plot with x-values and corresponding distances for the first set of files
plt.plot(x_values_1, distances_1, marker='o', linestyle='-', label='ACS 1')

# Create a line plot with x-values and corresponding distances for the second set of files
plt.plot(x_values_2, distances_2, marker='s', linestyle='-', label='ACS 2')

plt.xlabel("Map")
plt.ylabel("Distance (m)")
plt.title("Distance vs. Maps")

# Customize x-axis labels with file names for the first set of files
tick_positions = range(0, len(x_values_1), 10)

plt.xticks(tick_positions, tick_positions, rotation=0)
# Add a legend
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()
