#!/usr/bin/env python3
import os
import sys
import pandas as pd

path = None
if (len(sys.argv) != 2):
    print("Default output directory: khm2_ws/src/khm2_verification_pipeline/data")
    path = os.path.join(os.path.curdir, "../data")
    print(path)
    # sys.exit(1)
elif not os.path.isdir(sys.argv[1]):
    print("Error: <folder_with_bag_files> must be a folder")
    sys.exit(1)
else:
    path = os.path.abspath(sys.argv[1])


for file in os.listdir(path):
    if not file.endswith(".bag"):
        continue
    file_path = os.path.join(path, file)
    file_name_no_ext = str.rstrip(os.path.basename(file_path), ".bag")
    output_dir_path = f"{os.path.dirname(file_path)}/{file_name_no_ext}_csv"
    extension = file.split(".")[-1]

    # Check if the folder exists
    if not os.path.exists(output_dir_path):
        # If it doesn't exist, create it
        os.mkdir(output_dir_path)

    cmd1 = f"rostopic echo -b {file_path} -p /gazebo/model_states > {output_dir_path}/{file_name_no_ext}_model_states.csv"
    cmd2 = f"rostopic echo -b {file_path} -p /joint_states > {output_dir_path}/{file_name_no_ext}_joint_states.csv"
    cmd3 = f"rostopic echo -b {file_path} -p /imu > {output_dir_path}/{file_name_no_ext}_imu.csv"
    cmd4 = f"rostopic echo -b {file_path} -p /front_left_torque > {output_dir_path}/{file_name_no_ext}_front_left_torque.csv"
    cmd5 = f"rostopic echo -b {file_path} -p /front_right_torque > {output_dir_path}/{file_name_no_ext}_front_right_torque.csv"
    cmd6 = f"rostopic echo -b {file_path} -p /back_left_torque > {output_dir_path}/{file_name_no_ext}_back_left_torque.csv"
    cmd7 = f"rostopic echo -b {file_path} -p /back_right_torque > {output_dir_path}/{file_name_no_ext}_back_right_torque.csv"
    print(f"processing and outputting to csv for {file_name_no_ext}.bag")
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_model_states.csv"):
        os.system(cmd1)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_joint_states.csv"):
        os.system(cmd2)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_imu.csv"):
        os.system(cmd3)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_front_left_torque.csv"):
        os.system(cmd4)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_front_right_torque.csv"):
        os.system(cmd5)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_back_left_torque.csv"):
        os.system(cmd6)
    if not os.path.exists(f"{output_dir_path}/{file_name_no_ext}_back_right_torque.csv"):
        os.system(cmd7)
    print(f"done outputting csv")
    print(f"compiling csv data")
    csv_path_partial = os.path.join(output_dir_path, file_name_no_ext)

    # model_states: pose AND twist
    # joint_states: wheel angular velocity
    # imu: acceleration, gyroscope
    # torque csvs: torques of each wheel
    # load the csv
    state_df = pd.read_csv(csv_path_partial + "_model_states.csv")

    # find the index of the rover
    rover_idx = 0
    for col in state_df.columns:
        if "field.name" in col:
            if state_df[col][0] == "khm2":
                break
            else:
                rover_idx += 1
    print(rover_idx)

    # delete all columns unrelated to the rover's data
    for col in state_df.columns:
        if not col == "%time" and not str(rover_idx) in col:
            del state_df[col]
    del state_df[f"field.name{rover_idx}"]
    
    # rename the columns
    temp_cols = state_df.columns
    renamed_cols = []
    for col in temp_cols:
        if col == "%time":
            renamed_cols.append("%time")
            continue
        s = col.split('.')
        renamed_cols.append(f"{s[1][:-1]}.{'.'.join(s[2:])}")
    state_df.columns = renamed_cols
        
    print(state_df.columns)

