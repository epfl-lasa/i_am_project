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

def parse_value(cell):
    # convert string to float 
    return float(cell)


def plot_robot_data(csv_file, show_plot=True):

    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file, 
                     converters={'RosTime' : parse_value, 'JointPosition': parse_list, 'JointVelocity': parse_list, 'EEF_Position': parse_list, 
                                 'EEF_Orientation': parse_list, 'EEF_Velocity': parse_list, 'Inertia': parse_list, 'HittingFlux': parse_value})
                    #  dtype={'RosTime': 'float64'})

    # Get the 'Time' column as datetime
    df['RosTime'] = pd.to_datetime(df['RosTime'], unit='s')

    print(df.head())

    # Labels for the coordinates
    coordinate_labels = ['x', 'y', 'z']

    # Create subplots
    fig, axs = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

    # Plot each element of 'JointPositions'
    for i in range(7):
        axs[0].plot(df['RosTime'], df['JointPosition'].apply(lambda x: x[i]), label=f'Joint {i+1}')

    # Plot each element of 'JointVelocities'
    for i in range(7):
        axs[1].plot(df['RosTime'], df['JointVelocity'].apply(lambda x: x[i]), label=f'Joint {i+1}')

    # Plot each element of 'EEF_Position'
    for i in range(3):
        axs[2].plot(df['RosTime'], df['EEF_Position'].apply(lambda x: x[i]), label=f'Axis {coordinate_labels[i]}')

    # Plot each element of 'EEF_Velocity'
    for i in range(3):
        axs[3].plot(df['RosTime'], df['EEF_Velocity'].apply(lambda x: x[i]), label=f'Axis {coordinate_labels[i]}')

    # Plot hitting flux
    axs[4].plot(df['RosTime'], df['HittingFlux'])


    # Customize the plots
    plt.title('Robot Data Over Time')
    axs[0].set_title('Joint Positions Over Time')
    axs[1].set_title('Joint Velocities Over Time')
    axs[2].set_title('EEF Position Over Time')
    axs[3].set_title('EEF Velocity Over Time')
    axs[3].set_title('Hitting Flux Over Time')

    axs[4].set_xlabel('Time [s]')

    axs[0].set_ylabel('Joint angle [rad]')
    axs[1].set_ylabel('Joint velocity [rad/s]')
    axs[2].set_ylabel('Position[m]')
    axs[3].set_ylabel('Speed [m/s]')
    axs[4].set_ylabel('Hitting Flux [m/s]')
    
    for ax in axs:
        ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
        ax.grid(True)

    if show_plot : plt.show()


def plot_object_data(csv_file, show_plot=True):
    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file,
                     converters={'RosTime' : parse_value, 'Position': parse_list})
                    #  dtype={'RosTime': 'float64'})

    # Convert the 'Time' column to datetime format
    df['RosTime'] = pd.to_datetime(df['RosTime'], unit='s')

    # Labels for the coordinates
    coordinate_labels = ['x', 'y', 'z']

    # Plot the data
    plt.figure(figsize=(12, 6))
    for i in range(3):
        plt.plot(df['RosTime'], df['Position'].apply(lambda x: x[i]), label=f'Axis {coordinate_labels[i]}')

    # Customize the plot
    plt.title('Object Data Over Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Position')
    plt.legend()
    plt.grid(True)

    if show_plot : plt.show()

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
    hit_number = 3
        
    # Example usage
    # process_timestamped_folders('/path/to/timestamped_folders')

    # test one plot
    path_to_robot_hit = path_to_data_airhockey + f"2024-01-11_15:57:49/IIWA_14_hit_{hit_number}.csv"
    path_to_object_hit = path_to_data_airhockey + f"2024-01-11_15:57:49/object_hit_{hit_number}.csv"
    plot_robot_data(path_to_robot_hit, show_plot=False)
    plot_object_data(path_to_object_hit)