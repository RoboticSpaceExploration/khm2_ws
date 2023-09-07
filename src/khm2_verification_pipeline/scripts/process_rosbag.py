#!/usr/bin/env python3
import os
import sys

if (len(sys.argv) != 2):
    print("Usage: python test.py <folder_with_bag_files>")
    sys.exit(1)
elif not os.path.isdir(sys.argv[1]):
    print("Error: <folder_with_bag_files> must be a folder")
    sys.exit(1)

path = os.path.abspath(sys.argv[1])

for file in os.listdir(path):
    if not file.endswith(".bag"):
        continue
    file_path = os.path.join(path, file)
    file_name_no_ext = str.rstrip(os.path.basename(file_path), ".bag")
    output_folder = f"{os.path.dirname(file_path)}/{file_name_no_ext}_csv"
    extension = file.split(".")[-1]

    # Check if the folder exists
    if not os.path.exists(output_folder):
        # If it doesn't exist, create it
        os.mkdir(output_folder)

    cmd1 = f"rostopic echo -b {file_path} -p /gazebo/model_states > {output_folder}/{file_name_no_ext}_model_states.csv"
    cmd2 = f"rostopic echo -b {file_path} -p /joint_states > {output_folder}/{file_name_no_ext}_joint_states.csv"
    cmd3 = f"rostopic echo -b {file_path} -p /imu > {output_folder}/{file_name_no_ext}_imu.csv"
    cmd4 = f"rostopic echo -b {file_path} -p /front_left_torque > {output_folder}/{file_name_no_ext}_front_left_torque.csv"
    cmd5 = f"rostopic echo -b {file_path} -p /front_right_torque > {output_folder}/{file_name_no_ext}_front_right_torque.csv"
    cmd6 = f"rostopic echo -b {file_path} -p /back_left_torque > {output_folder}/{file_name_no_ext}_back_left_torque.csv"
    cmd7 = f"rostopic echo -b {file_path} -p /back_right_torque > {output_folder}/{file_name_no_ext}_back_right_torque.csv"
    print("processing bag files and outputting to csv...")
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_model_states.csv"):
        os.system(cmd1)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_joint_states.csv"):
        os.system(cmd2)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_imu.csv"):
        os.system(cmd3)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_front_left_torque.csv"):
        os.system(cmd4)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_front_right_torque.csv"):
        os.system(cmd5)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_back_left_torque.csv"):
        os.system(cmd6)
    if not os.path.exists(f"{output_folder}/{file_name_no_ext}_back_right_torque.csv"):
        os.system(cmd7)
    print("done!")