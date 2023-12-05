import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# def parse_list(cell):
#     # Parse the comma-separated values as a list of floats
#     # return [float(value) for value in cell.strip().split('\t')]

def parse_list(cell):
    # Split the space-separated values and parse them as a list of floats
    return [float(value) for value in cell.split()]

def plot_robot_data(csv_file):

    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file, 
                     converters={'JointPosition': parse_list, 'JointVelocity': parse_list, 'EEF_Position': parse_list, 'EEF_Velocity': parse_list},
                     dtype={'RosTime': 'float64'})

    print(df.head())

    # Get the 'Time' column as datetime
    # df['RosTime'] = pd.to_datetime(df['RosTime'], unit='s')

    # Create subplots
    fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)

    # Plot each element of 'JointPositions'
    for i in range(7):
        axs[0].plot(df['RosTime'], df['JointPosition'].apply(lambda x: x[i]), label=f'Joint {i+1}')

    # Plot each element of 'JointVelocities'
    for i in range(7):
        axs[1].plot(df['RosTime'], df['JointVelocity'].apply(lambda x: x[i]), label=f'Joint {i+1}')

    # Plot each element of 'EEF_Position'
    for i in range(3):
        axs[2].plot(df['RosTime'], df['EEF_Position'].apply(lambda x: x[i]), label=f'Coordinate {i+1}')

    # Plot each element of 'EEF_Velocity'
    for i in range(3):
        axs[3].plot(df['RosTime'], df['EEF_Velocity'].apply(lambda x: x[i]), label=f'Coordinate {i+1}')

    # Customize the plots
    axs[0].set_title('Joint Positions Over Time')
    axs[1].set_title('Joint Velocities Over Time')
    axs[2].set_title('EEF Position Over Time')
    axs[3].set_title('EEF Velocity Over Time')

    axs[3].set_xlabel('Time')
    
    for ax in axs:
        ax.set_ylabel('Values')
        ax.legend()
        ax.grid(True)

    plt.show()


def plot_object_data(csv_file):
    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file)

    # Convert the 'Time' column to datetime format
    df['RosTime'] = pd.to_datetime(df['RosRosTime'])

    # Plot the data
    plt.figure(figsize=(12, 6))
    plt.plot(df['RosTime'], df['Position'], label='Position')

    # Customize the plot
    plt.title('Robot Data Over Time')
    plt.xlabel('Time')
    plt.ylabel('Values')
    plt.legend()
    plt.grid(True)
    plt.show()

def process_timestamped_folders(root_folder):
    for folder in os.listdir(root_folder):
        folder_path = os.path.join(root_folder, folder)
        if os.path.isdir(folder_path) and len(folder) == 19 and folder[10] == '_':
            # Check if the folder name matches the timestamp format
            timestamp = folder.replace('_', ' ')
            try:
                datetime.strptime(timestamp, '%Y-%m-%d %H:%M:%S')
            except ValueError:
                continue

            # Iterate through CSV files in the folder
            for file in os.listdir(folder_path):
                if file.startswith('iiwa'):
                    file_path = os.path.join(folder_path, file)
                    plot_robot_data(file_path)
                if file.startswith('object'):
                    file_path = os.path.join(folder_path, file)
                    plot_robot_data(file_path)

if __name__== "__main__" :

    # path_to_data_airhockey = "/home/ros/ros_ws/src/i_am_project/data/airhockey/"
    path_to_data_airhockey = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/data/airhockey/"
        
    # Example usage
    # process_timestamped_folders('/path/to/timestamped_folders')

    # test one plot
    path_to_file = path_to_data_airhockey + "2023-12-05_09:55:21/iiwa_7_hit_1.csv"
    print(path_to_file)
    plot_robot_data(path_to_file)