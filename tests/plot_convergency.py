import os
import glob
import matplotlib.pyplot as plt
import numpy as np

path ="/home/esther/catkin_ws/src/acs_route_planning/tests/conv_logs"

def get_all_files(directory):
    all_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            all_files.append(file_path)
    return all_files

def read_log_file(file_path):
    with open(file_path, 'r') as file:
        distances = []
        for line in file:
            try:
                distance_value = float(line.strip().split('-')[-1])
                distances.append(distance_value)
            except ValueError:
                # Skip lines that don't contain a valid numerical value
                pass
    return distances


def plot_graph(iterations, average_distances):
    plt.plot(iterations, average_distances, marker='o')
    plt.title('Distance vs Iteration')
    plt.xlabel('Iteration')
    plt.ylabel('Average Distance')
    plt.show()

def main():
    # Get all log files in the current directory
    log_files = get_all_files(path)

    if not log_files:
        print("No log files found in the current directory.")
        return

    all_distances = []

    for log_file in log_files:
        distances = read_log_file(log_file)
        all_distances.append(distances)

    # Calculate the average distance for each iteration
    num_iterations = len(all_distances[0])  # Assuming all log files have the same number of iterations
    average_distances = [np.mean([distances[i] for distances in all_distances]) for i in range(num_iterations)]

    # Plot the graph
    iterations = list(range(1, num_iterations + 1))
    plot_graph(iterations, average_distances)

if __name__ == "__main__":
    main()
