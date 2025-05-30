import os
import matplotlib.pyplot as plt
import csv
import argparse



def read_coordinates_from_file(file_path):
    """
    Reads coordinates from a CSV file and returns lists of time, x, and y values.
    """
    times, x_coords, y_coords = [], [], []
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip the header row
        for row in reader:
            times.append(float(row[0]))
            x_coords.append(float(row[1]))
            y_coords.append(float(row[2]))
            # angle.append(float(row[3]))
    return times, x_coords, y_coords

def plot_graphs_from_files(directory_path, source_filenames):
    """
    Reads coordinate data from CSV files in the given directory and plots graphs.
    """
    plt.figure(figsize=(12, 8))

    for source in source_filenames:
        file_path = os.path.join(directory_path, f"{source}.csv")
        if not os.path.exists(file_path):
            print(f"File {file_path} does not exist. Skipping...")
            continue

        # Read data from the file
        times, x_coords, y_coords = read_coordinates_from_file(file_path)

        # Plot X vs Time
        plt.subplot(3, 1, 1)
        plt.plot(times, x_coords, label=f'{source} X')

        # Plot Y vs Time
        plt.subplot(3, 1, 2)
        plt.plot(times, y_coords, label=f'{source} Y')

        # Plot X vs Y
        plt.subplot(3, 1, 3)
        plt.plot(x_coords, y_coords, label=f'{source}')

    # Configure the subplots
    plt.subplot(3, 1, 1)
    plt.title('X Coordinates Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('X Coordinate')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.title('Y Coordinates Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Coordinate')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.title('X vs Y Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()

    plt.tight_layout()

    # Save the plot to a file instead of showing it
    plt.savefig(directory_path + '/plot.png')  # Save as PNG file

    # Show the plot
    plt.show()

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Plot graphs from coordinate log files.")
    parser.add_argument("directory", type=str, help="The directory containing the coordinate log files (e.g., '2025.03.10-12:00').")
    args = parser.parse_args()

    # Base log directory
    base_log_directory = "/home/orangepi/astabot/data/coordinate_log/"
    log_directory = os.path.join(base_log_directory, args.directory)

    # Check if the directory exists
    if not os.path.exists(log_directory):
        print(f"Directory {log_directory} does not exist. Please provide a valid directory.")
        exit(1)

    # Source filenames
    source_files = ['odom', 'amcl', 'tf']  # Replace with your actual source filenames

    # Plot graphs
    plot_graphs_from_files(log_directory, source_files)
