import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time
import re

# PARSING FUNCTIONS
def parse_list(cell):
    # Split the space-separated values and parse them as a list of floats
    return [float(value) for value in cell.split()]

def parse_value(cell):
    # convert string to float 
    return float(cell)

def get_flux_and_inertia_at_hit(csv_file, hit_time, show_print=False, get_max_values=False):
    
    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file, skiprows=1,
                     converters={'RosTime' : parse_value, 'JointPosition': parse_list, 'JointVelocity': parse_list, 'JointEffort': parse_list, 
                                 'TorqueCmd': parse_list, 'EEF_Position': parse_list, 'EEF_Orientation': parse_list, 'EEF_Velocity': parse_list, 
                                 'EEF_DesiredVelocity': parse_list, 'Inertia': parse_list, 'HittingFlux': parse_value})
                    #  dtype={'RosTime': 'float64'})
    
    # Get DF of values after hit
    post_hit_df = df[(df['RosTime']-hit_time) >= 0]

    # Check that we recorded at this time
    if post_hit_df.empty:
        print(f"ERROR Robot data stops before Hit Time for {csv_file}")
        return 0,0

    # Get flux at closest hit time 
    flux_at_hit = post_hit_df.iloc[0]['HittingFlux']

    # Get directional inertia at hit time
    # NOTE : Inertia recorded is actually Inertia Task Position INVERSE
    inertia_at_hit = post_hit_df.iloc[0]['Inertia']

    des_direction = np.array([[0.0], [1.0], [0.0]])
    dir_inertia_at_hit = 1/(des_direction.T @ np.reshape(inertia_at_hit, (3,3)) @ des_direction)[0,0]

    if get_max_values : 
        # Get max normed vel
        normed_vel = df['EEF_Velocity'].apply(lambda x: np.linalg.norm(x))
        max_vel = normed_vel.max()
        
        # Get max flux
        max_flux = df['HittingFlux'].abs().max()
    
    if show_print: 
        # Define set values from first row
        df_top_row = pd.read_csv(csv_file, nrows=1, header=None)
        top_row_list = df_top_row.iloc[0].to_list()
        des_flux = top_row_list[1]
        des_pos = parse_list(top_row_list[5])
        recorded_hit_time = top_row_list[3]
        # print(f"Desired Flux: {des_flux} \n Desired Pos: [{des_pos[0]:.3f}, {des_pos[1]:.3f}, {des_pos[2]:.3f}] \n Hit Time: {pd.to_datetime(recorded_hit_time, unit='s')}")

        # Make title string
        filename = os.path.basename(csv_file)
        filename_without_extension = os.path.splitext(filename)[0]
        parts = filename_without_extension.split('_')

        print(f"Hit #{parts[3]}, IIWA_{parts[1]} \n Desired Flux: {des_flux} \n Hitting Flux: {flux_at_hit:.4f}")
        print(f" Real Hit time : {hit_time} \n Recorded Hit Time : {pd.to_datetime(recorded_hit_time, unit='s')}")
    
    if get_max_values : 
        return max_flux, max_vel

    else: 
        return flux_at_hit, dir_inertia_at_hit

def get_impact_time_from_object(csv_file, show_print=False, return_indexes=False):    
    # Reads object csv file and returns impact time OR indexes for before_impact, after_impact, stop moving

    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file,
                     converters={'RosTime' : parse_value, 'Position': parse_list})
                
    ### SOLUTION TO DEAL WITH RECORDING OF MANUAL MOVEMENT 
    # Use derivative to find changes in speed 
    derivative_threshold = 0.05

    # find start and end index by using derivative in x axis -- NOTE : ASSUME MOVEMENT IN X AXIS
    x_values =  df['Position'].apply(lambda x: x[0])
    df['derivative'] = x_values.diff() / df['RosTime'].diff()

    # remove zeros
    filtered_df = df[df['derivative'] != 0.0].copy()

    # get start and end index
    idx_start_moving =  (filtered_df['derivative'].abs() > derivative_threshold).idxmax() # detect 1st time derivative is non-zero
    idx_stop_moving = (filtered_df['derivative'].loc[idx_start_moving:].abs() < derivative_threshold).idxmax() # detect 1st time derivative comes back to zero
    idx_before_impact = idx_start_moving-1 # time just before impact - 10ms error due to Motive streaming at 120Hz 

    hit_time = df['RosTime'].iloc[idx_before_impact] # HIT TIME as float 

    if show_print: 
        df['RosTime'] = pd.to_datetime(df['RosTime'], unit='s')
        print(f"Start moving from {df['Position'].iloc[idx_before_impact]} at {df['RosTime'].iloc[idx_before_impact]}")
        print(f"Stop moving from {df['Position'].iloc[idx_stop_moving]} at {df['RosTime'].iloc[idx_stop_moving]}")

    if not return_indexes: 
        return hit_time
    elif return_indexes:
        if show_print : print("Return object movement indexes")
        return idx_before_impact, idx_start_moving, idx_stop_moving

def get_distance_travelled(csv_file, return_distance_in_x=False, show_print=False, show_hit=True):
    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(csv_file,
                     converters={'RosTime' : parse_value, 'Position': parse_list})
                    #  dtype={'RosTime': 'float64'})

    # # Make title string
    filename = os.path.basename(csv_file)
    filename_without_extension = os.path.splitext(filename)[0]
    parts = filename_without_extension.split('_')

    idx_before_impact, idx_start_moving, idx_stop_moving = get_impact_time_from_object(csv_file, return_indexes=True)

    ### Get distance in X = axis of hit in optitrack frame
    distance_in_x = df['Position'].iloc[idx_before_impact][0]- df['Position'].iloc[idx_stop_moving][0]
    #### Get distance in norm 
    norm_distance = np.linalg.norm(np.array(df['Position'].iloc[idx_before_impact])-np.array(df['Position'].iloc[idx_stop_moving]))
    
    if show_print :
        if show_hit:
            print(f"Hit #{parts[2]}, Object \n")
        print(f" Distance in Y-axis : {distance_in_x:.3f} \n Normed Distance : {norm_distance:.3f}")

    if return_distance_in_x : 
        return distance_in_x
    else : 
        return norm_distance

def get_info_at_hit_time(robot_csv, object_csv):

    # get hit time 
    hit_time = get_impact_time_from_object(object_csv)

    # get flux, inertia on hit

    hitting_flux, hitting_dir_inertia = get_flux_and_inertia_at_hit(robot_csv, hit_time)

    # get distance travelled
    distance_travelled = get_distance_travelled(object_csv, show_print=False)

    # get recording session, iiwa number and hit number from robot_csv name 
    recording_session = os.path.basename(os.path.dirname(robot_csv))
    filename = os.path.basename(robot_csv)
    filename_without_extension = os.path.splitext(filename)[0]
    parts = filename_without_extension.split('_')
    iiwa_number = parts[1]
    hit_number = parts[3]

    # Should be ordered in the same way as output file columns
    return [recording_session, hit_number, iiwa_number,  distance_travelled, hitting_flux, hitting_dir_inertia]

def process_data_to_one_file(recording_sessions, output_filename="test.csv"):
    
    path_to_data_airhockey = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/data/airhockey/"
    output_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/data/airhockey_processed/" + output_filename

    output_df = pd.DataFrame(columns=["RecSession","HitNumber","IiwaNumber","DistanceTraveled","HittingFlux","HittingInertia"])

    start_time= time.time()
    # process each rec_sess folder 
    for rec_sess in recording_sessions: 
        
        folder_name = os.path.join(path_to_data_airhockey,rec_sess)

        # Iterate over files in the folder and write filenames to dictionary
        hit_files = {}
        for filename in os.listdir(folder_name):
            # Check if the file matches the pattern
            match = re.match(r'(?:IIWA_\d+_hit_|object_hit_)(\d+)\.csv', filename)
            if match:
                hit_number = int(match.group(1))
                # Append the file to the corresponding hit_number key in the dictionary
                hit_files.setdefault(hit_number, []).append(os.path.join(folder_name, filename))

        # Iterate over pairs of files with the same hit number
        for hit_number, files in hit_files.items():
            # Check if there are at least two files with the same hit number
            if len(files) == 2:
                
                files.sort() # Sort the files to process them in order -> robot_csv, then object_csv
                
                ## DEBUG print(files[0])

                # process each "hit_number" set 
                output_df.loc[len(output_df)]= get_info_at_hit_time(files[0], files[1])
        

            else :
                print(f"ERROR : Wrong number of files, discarding the following : \n {files}")

        print(f"FINISHED {rec_sess}")

    # HACK - remove lines where flux AND inertia are 0 -> lines where we didn't record robot data at hit time
    clean_output_df = output_df[output_df['HittingFlux'] != 0.0].copy()
    print(f"Removing {len(output_df[output_df['HittingFlux'] == 0.0].index)} datapoints that were not recorded correctly")

    # Save output df as .csv
    clean_output_df.to_csv(output_path)
    
    print(f"Took {time.time()-start_time} seconds \nProcessed impact info for folders : {recording_sessions}")

    return


if __name__== "__main__" :
   
    ### Processing variables 
    folders_to_process = ["2024-03-05_12:20:48","2024-03-05_12:28:21","2024-03-05_14:04:43","2024-03-05_14:45:46","2024-03-05_15:19:15","2024-03-05_15:58:41",
                          "2024-03-06_12:30:55", "2024-03-06_13:40:26","2024-03-06_13:52:53","2024-03-06_15:03:42" ]

    process_data_to_one_file(folders_to_process, output_filename="all_data_march_test.csv")

