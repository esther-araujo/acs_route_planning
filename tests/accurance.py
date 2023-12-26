import os

# Define the directory containing your log files
log_directory = "tests/acs_logs"

# Initialize a counter for files with goalFounded set to true
goal_founded_count = 0

# Loop through each file in the directory
for filename in os.listdir(log_directory):
    if filename.endswith(".log"):  # Assuming log files have a .log extension
        with open(os.path.join(log_directory, filename), "r") as file:
            content = file.read()
            if "goalFounded: true" in content:
                goal_founded_count += 1
            else:
                print(filename)

print(f"Number of log files with goalFounded set to true: {goal_founded_count}")
