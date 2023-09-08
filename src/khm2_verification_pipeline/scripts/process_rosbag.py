#!/usr/bin/env python3
import os
import sys
import pandas as pd
import numpy as np

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

    """
        Preprocesing goal: merge all data into one clean csv

        model_states: pose AND twist
        joint_states: wheel angular velocity
        imu: acceleration, gyroscope
        torque csvs: torques of each wheel
        load the csv
    """
    state_df = pd.read_csv(csv_path_partial + "_model_states.csv")
    imu_df = pd.read_csv(csv_path_partial + "_imu.csv")
    joint_states_df = pd.read_csv(csv_path_partial + "_joint_states.csv")
    fl_torque_df = pd.read_csv(csv_path_partial + "_front_left_torque.csv")
    fr_torque_df = pd.read_csv(csv_path_partial + "_front_right_torque.csv")
    bl_torque_df = pd.read_csv(csv_path_partial + "_back_left_torque.csv")
    br_torque_df = pd.read_csv(csv_path_partial + "_back_right_torque.csv")


    # ============= model_states csv needs special processing ===============
    # find the index of the rover
    rover_idx = 0
    for col in state_df.columns:
        if "field.name" in col:
            if state_df[col][0] == "khm2":
                break
            else:
                rover_idx += 1

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
    # ============== model_states csv ===================

    # ============== clean joint_states csv ==============
    for col in joint_states_df.columns:
        if "effort" in col:
            del joint_states_df[col]
    # ============== clean joint_states csv ==============

    def delete_field_from_col(df: pd.DataFrame):
        renamed_cols = []
        for col in df.columns:
            if "time" in col:
                renamed_cols.append(col)
                continue
            renamed_cols.append('.'.join(col.split('.')[1:])) # delete "field" part at start
        df.columns = renamed_cols
    delete_field_from_col(imu_df)
    delete_field_from_col(joint_states_df)
    delete_field_from_col(fl_torque_df)
    delete_field_from_col(fr_torque_df)
    delete_field_from_col(bl_torque_df)
    delete_field_from_col(br_torque_df)

    # ============== clean wheel torque dataframes ==============
    def clean_torque_dataframes(d: pd.DataFrame, prepend_val):
        renamed_cols = []
        for col in d.columns:
            if "time" in col:
                renamed_cols.append(col)
            else:
                renamed_cols.append(prepend_val + col)
        d.columns = renamed_cols
    clean_torque_dataframes(fl_torque_df, "fl.")
    clean_torque_dataframes(fr_torque_df, "fr.")
    clean_torque_dataframes(bl_torque_df, "bl.")
    clean_torque_dataframes(br_torque_df, "br.")
    # ============== clean wheel torque dataframes ==============

    # ============== clean joint state dataframes ==============
    # 2 = front left, 3 = front right, 0 = back left, 1 = back right
    # print(joint_states_df["name2"][0]) # front left
    # print(joint_states_df["name3"][0]) # front right
    # print(joint_states_df["name0"][0]) # back left
    # print(joint_states_df["name1"][0]) # back right
    new_joint_states_df = pd.DataFrame()
    joint_states_df["front_left_velocity"] = joint_states_df["velocity2"]
    joint_states_df["front_right_velocity"] = joint_states_df["velocity3"]
    joint_states_df["back_left_velocity"] = joint_states_df["velocity0"]
    joint_states_df["back_right_velocity"] = joint_states_df["velocity1"]
    del joint_states_df["velocity0"]
    del joint_states_df["velocity1"]
    del joint_states_df["velocity2"]
    del joint_states_df["velocity3"]
    del joint_states_df["name0"]
    del joint_states_df["name1"]
    del joint_states_df["name2"]
    del joint_states_df["name3"]
    # ============== clean joint state dataframes ==============

    all_dfs = [state_df, imu_df, joint_states_df, fl_torque_df, fr_torque_df, bl_torque_df, br_torque_df]

    # ============= convert time from nanoseconds to seconds ===============
    for d in all_dfs:
        for col in d.columns:
            if "stamp" in col or col == "%time":
                d[col] = d[col].map(lambda x: x / 1_000_000_000)
                new_times = []
                start = d[col][0]
                for time in d[col]:
                    new_times.append(time - start)
                d[col] = new_times


    # ============= convert time from nanoseconds to seconds ===============

    df = pd.concat([state_df, imu_df, joint_states_df, fl_torque_df, fr_torque_df, bl_torque_df, br_torque_df], axis=1, join='inner')
    df_names = ["model_states", "imu", "joint_states", "fl_torque", "fr_torque", "bl_torque", "br_torque"]

    avg_hz = []
    for d in all_dfs:
        points = []
        first = d["%time"][0]
        prev = 0
        for i, time in enumerate(d["%time"]):
            time -= first
            if i != 0:
                points.append((time) - prev)
            else:
                points.append(time)
            prev = time
        avg_hz.append(1000 / (np.mean(points) * 1000)) # convert Hz, msgs per sec
    for i, t in enumerate(avg_hz):
        if abs(t - 1000) > 3:
            print(f"Warning: {df_names[i]} has average sample rate of {t} Hz. Ideally expecting about 1000Hz")

    # ======== done cleaning dataframes =========
    # print keys of each dataframe
    def print_df_keys(df: pd.DataFrame):
        col_width = max(len(word) for word in df.columns) + 2  # padding
        max_per_col = 3
        i = 0
        for col in df.columns:
            if i == max_per_col:
                print()
                i = 0
            i += 1
            print(f"{col:{col_width}}", end="")
        print("\n")

    print(f"========== rover pose and twist ==========")
    print_df_keys(state_df)

    print(f"========== imu ===========================")
    print_df_keys(imu_df)

    print(f"========== wheel velocities ==============")
    print_df_keys(joint_states_df)

    print(f"========== wheel torques ==============")
    print_df_keys(fl_torque_df)
    print_df_keys(fr_torque_df)
    print_df_keys(bl_torque_df)
    print_df_keys(br_torque_df)