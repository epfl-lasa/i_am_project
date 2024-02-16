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
    df = pd.read_csv(csv_file, skiprows=1,
                     converters={'RosTime' : parse_value, 'JointPosition': parse_list, 'JointVelocity': parse_list, 'EEF_Position': parse_list, 
                                 'EEF_Orientation': parse_list, 'EEF_Velocity': parse_list, 'Inertia': parse_list, 'HittingFlux': parse_value})
                    #  dtype={'RosTime': 'float64'})
    
    df_top_row = pd.read_csv(csv_file, nrows=1)
    print(df_top_row)

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

    # Make title string
    filename = os.path.basename(csv_file)
    filename_without_extension = os.path.splitext(filename)[0]
    parts = filename_without_extension.split('_')
    title_str = f"Robot data for iiwa {parts[1]}, hit #{parts[3]}" #filename_without_extension.replace('_', ' ')

    # Customize the plots
    plt.suptitle(title_str)
    axs[0].set_title('Joint Positions Over Time')
    axs[1].set_title('Joint Velocities Over Time')
    axs[2].set_title('EEF Position Over Time')
    axs[3].set_title('EEF Velocity Over Time')
    axs[4].set_title('Hitting Flux Over Time')

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


def plot_actual_vs_des(csv_file, inverse_effort=True, show_plot=True):

    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file, skiprows=1,
                     converters={'RosTime' : parse_value, 'JointPosition': parse_list, 'JointVelocity': parse_list, 'JointEffort': parse_list, 
                                 'TorqueCmd': parse_list, 'EEF_Position': parse_list, 'EEF_Orientation': parse_list, 'EEF_Velocity': parse_list, 
                                 'EEF_DesiredVelocity': parse_list, 'Inertia': parse_list, 'HittingFlux': parse_value})
                    #  dtype={'RosTime': 'float64'})
    
    # Define set values from first row
    df_top_row = pd.read_csv(csv_file, nrows=1, header=None)
    top_row_list = df_top_row.iloc[0].to_list()
    des_flux = top_row_list[1]
    des_pos = parse_list(top_row_list[5])
    recorded_hit_time = top_row_list[3]
    print(f"Desired Flux: {des_flux} \n Desired Pos: [{des_pos[0]:.3f}, {des_pos[1]:.3f}, {des_pos[2]:.3f}] \n Hit Time: {pd.to_datetime(recorded_hit_time, unit='s')}")

    # Make title string
    filename = os.path.basename(csv_file)
    filename_without_extension = os.path.splitext(filename)[0]
    parts = filename_without_extension.split('_')
    # title_str = f"Robot data for iiwa {parts[1]}, hit #{parts[3]}" #filename_without_extension.replace('_', ' ')

    # Get the 'Time' column as datetime
    df['RosTime'] = pd.to_datetime(df['RosTime'], unit='s')

    # Labels for the coordinates
    coordinate_labels = ['x', 'y', 'z']

    # Invers Effort for easier to read plots
    if inverse_effort: effort_factor = -1
    else: effort_factor = 1

    # Plot JointEffort vs TorqueCmd
    fig, axs = plt.subplots(7, 1, figsize=(15, 12), sharex=True)
    for i in range(7):
        axs[i].plot(df['RosTime'], effort_factor*df['JointEffort'].apply(lambda x: x[i]), label=f'Effort')
        axs[i].plot(df['RosTime'], df['TorqueCmd'].apply(lambda x: x[i]), color='r', linestyle='--', label=f'Torque Cmd')
        axs[i].set_title(f'Joint{i+1}')
        axs[i].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
        axs[i].grid(True)
    axs[i].set_xlabel('Time [s]')
    fig.suptitle(f"Effort vs Cmd : iiwa {parts[1]}, hit #{parts[3]}, flux {des_flux}")
    fig.tight_layout(rect=(0.01,0.01,0.99,0.99))


    # Plot EEF_Velocities vs Desired Velocities
    fig, axs = plt.subplots(3, 1, figsize=(9, 12), sharex=True)
    for i in range(3):
        axs[i].plot(df['RosTime'], df['EEF_Velocity'].apply(lambda x: x[i]), label=f'Velocity')
        axs[i].plot(df['RosTime'], df['EEF_DesiredVelocity'].apply(lambda x: x[i]), color='r',linestyle='--', label=f'Desired')
        axs[i].set_title(f'Axis {coordinate_labels[i]}')
        axs[i].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
        axs[i].grid(True)
    axs[i].set_xlabel('Time [s]')
    fig.suptitle(f"EEF Velocities vs Desired : iiwa {parts[1]}, hit #{parts[3]}")
    fig.tight_layout(rect=(0.01,0.01,0.99,0.99))


    # Plot EEF_Position vs Desired Psosiont
    fig, axs = plt.subplots(3, 1, figsize=(9, 12), sharex=True)
    for i in range(3):
        axs[i].plot(df['RosTime'], df['EEF_Position'].apply(lambda x: x[i]), label=f'Position')
        axs[i].axhline(y=des_pos[i], color='r', linestyle='--')
        axs[i].set_title(f'Axis {coordinate_labels[i]}')
        axs[i].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
        axs[i].grid(True)
    axs[i].set_xlabel('Time [s]')
    fig.suptitle(f"EEF Position vs Desired : iiwa {parts[1]}, hit #{parts[3]}")
    fig.tight_layout(rect=(0.01,0.01,0.99,0.99))


    # Plot Flux
    fig, ax = plt.subplots(1, 1, figsize=(10, 4), sharex=True)
    ax.plot(df['RosTime'], df['HittingFlux'])
    ax.axhline(y=des_flux, color='r', linestyle='--')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Hitting flux [m/s]')
    ax.grid(True)
    fig.suptitle(f"Hitting Flux : iiwa {parts[1]}, hit #{parts[3]}")
    fig.tight_layout(rect=(0.01,0.01,0.99,0.99))

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

    # Make title string
    filename = os.path.basename(csv_file)
    filename_without_extension = os.path.splitext(filename)[0]
    parts = filename_without_extension.split('_')
    title_str = f"Object data for hit #{parts[2]}"

    # Customize the plot
    plt.title(title_str)
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
    
    folder_name = "2024-02-08_14:04:37"
    hit_number = 6
    iiwa_number = 7

    # test one plot
    path_to_robot_hit = path_to_data_airhockey + f"{folder_name}/IIWA_{iiwa_number}_hit_{hit_number}.csv"
    path_to_object_hit = path_to_data_airhockey + f"{folder_name}/object_hit_{hit_number}.csv"

    plot_actual_vs_des(path_to_robot_hit)
    # plot_robot_data(path_to_robot_hit, show_plot=False)
    # plot_object_data(path_to_object_hit)