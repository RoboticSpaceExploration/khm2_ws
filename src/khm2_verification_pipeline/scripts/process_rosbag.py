#!/usr/bin/env python3
import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation
import analysis
from analysis import data

path = None
if (len(sys.argv) != 2):
    print("Default output directory: khm2_ws/src/khm2_verification_pipeline/data")
    path = os.path.join(os.path.dirname(__file__),"../data")
    print(path)
    # sys.exit(1)
elif not os.path.isdir(sys.argv[1]):
    print("Error: <folder_with_bag_files> must be a folder")
    sys.exit(1)
else:
    path = os.path.abspath(sys.argv[1])

class data:
    def __init__(self, vicon_time_cut, position_x_cut, position_y_cut, position_z_cut,
                rotation_x_cut, rotation_y_cut, rotation_z_cut, rotation_w_cut, 
                rover_time_cut, accel_x_cut, accel_y_cut, accel_z_cut, 
                gyro_x_cut, gyro_y_cut, gyro_z_cut, 
                motor_rpm_1_cut, motor_rpm_2_cut, motor_rpm_3_cut, motor_rpm_4_cut,
                motor_current_1_cut, motor_current_2_cut, motor_current_3_cut, motor_current_4_cut):
        self.vicon_time_cut = vicon_time_cut
        self.position_x_cut = position_x_cut
        self.position_y_cut = position_y_cut
        self.position_z_cut = position_z_cut
        self.rotation_x_cut = rotation_x_cut
        self.rotation_y_cut = rotation_y_cut
        self.rotation_z_cut = rotation_z_cut
        self.rotation_w_cut = rotation_w_cut
        self.rover_time_cut = rover_time_cut
        self.accel_x_cut = accel_x_cut
        self.accel_y_cut = accel_y_cut
        self.accel_z_cut = accel_z_cut
        self.gyro_x_cut = gyro_x_cut
        self.gyro_y_cut = gyro_y_cut
        self.gyro_z_cut = gyro_z_cut
        self.motor_rpm_1_cut = motor_rpm_1_cut
        self.motor_rpm_2_cut = motor_rpm_2_cut
        self.motor_rpm_3_cut = motor_rpm_3_cut
        self.motor_rpm_4_cut = motor_rpm_4_cut
        self.motor_current_1_cut = motor_current_1_cut
        self.motor_current_2_cut = motor_current_2_cut
        self.motor_current_3_cut = motor_current_3_cut
        self.motor_current_4_cut = motor_current_4_cut

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

    print(f"========== DataFrame: state_df | rover pose and twist ==========")
    print_df_keys(state_df)

    print(f"========== DataFrame: imu_df | imu data ===========================")
    print_df_keys(imu_df)

    print(f"========== DataFrame: joint_state_df | wheel velocities ==============")
    print_df_keys(joint_states_df)

    print(f"==== DataFrames: fl_torque_df, fr_torque_df, bl_torque_df, br_torque_df =============")
    print(f"================ wheel torques ==============")
    print_df_keys(fl_torque_df)
    print_df_keys(fr_torque_df)
    print_df_keys(bl_torque_df)
    print_df_keys(br_torque_df)

    # import cadre data with 0 degrees

    # Slip vs sinkage 
    # slip vs angle

    # necessary
    # RMS error xyz across time
        # between samples
        # Useful for? 
            # represent steady state vs initial condition
    
    # nice to have
    # compare to ishigami's theory
        # drawbar pull
    # virtual bevameter?
        # sinkage pressure curve

    # tonight, traction equations
    # grouser modeling

    # mock results
    # 1. cadre initial conditions into sim
    # 2. sampling rate, run couple tests on titan
    # 3. analysis: RMS error, angular velocity, position, linear velocity, etc

    # RMS error will give idea
    # 

    # time_arr1 and val_arr1 assumed to have same length
    def downsample(time_arr1, time_arr2, val_arr2):
        result = []
        for t in time_arr1:
            i = (np.abs(time_arr2 - t)).argmin()
            result.append(val_arr2[i])
        return result

    fr_vel = joint_states_df["front_right_velocity"].to_numpy()
    joint_states_time = joint_states_df["%time"].to_numpy()
    fig = plt.figure(figsize=(20,10))
    ax1 = fig.add_subplot(4,4,1)
    ax1.plot(joint_states_time, fr_vel)
    plt.xlabel('time [sec]')
    plt.ylabel('front right wheel velocity [m/s]')

    state_df_time = state_df["%time"].to_numpy()
    posx_downsampled = downsample(joint_states_time, state_df_time, state_df["pose.position.x"].to_numpy())
    posy_downsampled = downsample(joint_states_time, state_df_time, state_df["pose.position.y"].to_numpy())
    posz_downsampled = downsample(joint_states_time, state_df_time, state_df["pose.position.z"].to_numpy())
    posx_downsampled = [-1 * x for x in posx_downsampled]
    posy_downsampled = [-1 * x for x in posy_downsampled]
    posz_downsampled = [-1 * x for x in posz_downsampled]

    ax2 = fig.add_subplot(4,4,2)
    ax2.plot(joint_states_time, posx_downsampled, label= 'x')
    plt.xlabel('time [sec]')
    plt.ylabel('x position [m]')
    plt.ylim(-1, 1)

    ax2 = fig.add_subplot(4,4,3)
    ax2.plot(joint_states_time, posy_downsampled, label= 'y')
    plt.xlabel('time [sec]')
    plt.ylabel('y position [m]')

    ax2 = fig.add_subplot(4,4,4)
    ax2.plot(joint_states_time, posz_downsampled, label= 'z')
    plt.xlabel('time [sec]')
    plt.ylabel('z position [m]')
    plt.ylim(-1, 1)

    # need linear velocity xyz and angular velocity xyz of rover
    sim_lin_x = downsample(joint_states_time, state_df_time, state_df["twist.linear.x"].to_numpy())
    sim_lin_y = downsample(joint_states_time, state_df_time, state_df["twist.linear.y"].to_numpy())
    sim_lin_z = downsample(joint_states_time, state_df_time, state_df["twist.linear.z"].to_numpy())
    sim_ang_x = downsample(joint_states_time, state_df_time, state_df["twist.angular.x"].to_numpy())
    sim_ang_y = downsample(joint_states_time, state_df_time, state_df["twist.angular.y"].to_numpy())
    sim_ang_z = downsample(joint_states_time, state_df_time, state_df["twist.angular.z"].to_numpy())


    quat_x = downsample(joint_states_time, state_df_time, state_df["pose.orientation.x"].to_numpy())
    quat_y = downsample(joint_states_time, state_df_time, state_df["pose.orientation.x"].to_numpy())
    quat_z = downsample(joint_states_time, state_df_time, state_df["pose.orientation.x"].to_numpy())
    quat_w = downsample(joint_states_time, state_df_time, state_df["pose.orientation.x"].to_numpy())
    quat_df = np.transpose(np.array([quat_x, quat_y, quat_z, quat_w]))

    # World space linear velocity of rover
    lin_vel_world = np.array(np.transpose([sim_lin_x, sim_lin_y, sim_lin_z]))
    ang_vel_world = np.array(np.transpose([sim_ang_x, sim_ang_y, sim_ang_z]))

    # get rotation matrix to convert to local space
    rotations = []
    for quat in quat_df:
        rotations.append(quaternion_matrix(quat)[:3,:3])
    
    # transform to local space for linear velocity 
    lin_vel_local = []
    ang_vel_local = []
    for i, rot in enumerate(rotations):
        lin_vel_local.append(np.matmul(np.linalg.inv(rot), lin_vel_world[i]))
        ang_vel_local.append(np.matmul(np.linalg.inv(rot), ang_vel_world[i]))

    lin_vel_local = np.transpose(lin_vel_local)
    lin_vel_x_local = lin_vel_local[0]
    lin_vel_y_local = lin_vel_local[1]
    lin_vel_z_local = lin_vel_local[2]

    ang_vel_local = np.transpose(ang_vel_local)
    ang_vel_x_local = ang_vel_local[0]
    ang_vel_y_local = ang_vel_local[1]
    ang_vel_z_local = ang_vel_local[2]
    
    def get_wheel_lin_vel(body_lin_vel, body_ang_vel, radius):
        result = []
        # index 0 = x, index 1 = y, index 2 = z
        for i, lin_vel in enumerate(body_lin_vel):
            result.append(np.cross(body_ang_vel[i], radius) + lin_vel)
        return result

    print("calculating slip ratios")
    fl_wheel_lin_vel = get_wheel_lin_vel(list(zip(lin_vel_x_local, lin_vel_y_local, lin_vel_z_local)), 
                                         list(zip(ang_vel_x_local, ang_vel_y_local, ang_vel_z_local)), 
                                         [-0.145, 0.145225, -0.1])
    fr_wheel_lin_vel = get_wheel_lin_vel(list(zip(lin_vel_x_local, lin_vel_y_local, lin_vel_z_local)), 
                                         list(zip(ang_vel_x_local, ang_vel_y_local, ang_vel_z_local)), 
                                         [0.145, 0.145225, -0.1])
    bl_wheel_lin_vel = get_wheel_lin_vel(list(zip(lin_vel_x_local, lin_vel_y_local, lin_vel_z_local)), 
                                         list(zip(ang_vel_x_local, ang_vel_y_local, ang_vel_z_local)), 
                                         [-0.145, -0.145225, -0.1])
    br_wheel_lin_vel = get_wheel_lin_vel(list(zip(lin_vel_x_local, lin_vel_y_local, lin_vel_z_local)), 
                                         list(zip(ang_vel_x_local, ang_vel_y_local, ang_vel_z_local)), 
                                         [0.145, -0.145225, -0.1])

    fl_slip = [1 - (lin_vel / joint_states_df["front_left_velocity"][i]) for i, lin_vel in enumerate(fl_wheel_lin_vel)]

    # print(fl_slip)


    ax2 = fig.add_subplot(4,4,6)
    ax2.plot(joint_states_time, posy_downsampled, label='y')
    plt.xlabel('time [sec]')
    plt.ylabel('linear velocity x [m/s]')
    # plt.ylim(-1, 1)
    lin_vel_y_range = abs(plt.ylim()[0] - plt.ylim()[1])

    ax2 = fig.add_subplot(4,4,5)
    ax2.plot(joint_states_time, posx_downsampled, label='x')
    plt.xlabel('time [sec]')
    plt.ylabel('linear velocity x [m/s]')
    center = (plt.ylim()[0] + plt.ylim()[1]) / 2
    plt.ylim(center - lin_vel_y_range, center + lin_vel_y_range)

    ax2 = fig.add_subplot(4,4,7)
    ax2.plot(joint_states_time, posz_downsampled, label='z')
    plt.xlabel('time [sec]')
    plt.ylabel('linear velocity x [m/s]')
    center = (plt.ylim()[0] + plt.ylim()[1]) / 2
    plt.ylim(center - lin_vel_y_range, center + lin_vel_y_range)
    # plt.ylim(-1, 1)

    # ax2 = fig.add_subplot(3,1,2)
    # ax2.plot(rover_time_cut, position_y_cut, label= 'y')
    # plt.xlabel('time [sec]')
    # plt.ylabel('y position [m]')

    # for type hinting
    def get_type_hinting(cadre_data) -> data:
        return cadre_data

    cadre_data, degrees_mapped_files = analysis.get_cadre_files()
    # get all data related to 0 degree inclination
    deg0_files = [file for file in cadre_data.keys() if degrees_mapped_files[file] == 0]
    print(deg0_files)

    deg0_file_test = cadre_data[deg0_files[0]]
    rover_time_cut = deg0_file_test.rover_time_cut

    # downsample rover data to nearest cadre time stamp
    
    # plt.show()

