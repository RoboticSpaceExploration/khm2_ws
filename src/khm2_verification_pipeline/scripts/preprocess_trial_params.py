#!/usr/bin/env python3

import rospkg
import json
import pickle
import os
import re # for parsing degrees of trial
import math
import numpy as np

# NOTE: MAKE SURE cut_data FOLDER EXISTS IN THE khm2_verification_pipeline/data FOLDER

# read cadre data file
# need this class for pickle, why so big
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

def generate_trial_params_json():
    rospack = rospkg.RosPack()
    output_file_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data/trial_params.json')
    cadre_data_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data/cut_data')

    cadre_data_files = []
    for file_name in os.listdir(cadre_data_path):
        if "refined" in file_name and file_name.endswith('.p'):
            cadre_data_files.append(file_name)

    json_trial_params = {}

    # for each pickle file, find angle, time elapsed, average top speed
    for file_name in cadre_data_files:
        # angle (degrees)
        regex = r"(\d+)DEG" # captures the number before DEG
        degrees = int(re.findall(regex, file_name)[0]) # take the first group

        # time elapsed
        trial = pickle.load(open(os.path.join(cadre_data_path, file_name), "rb"))

        vicon_time_cut      = trial.vicon_time_cut
        position_x_cut      = trial.position_x_cut
        position_y_cut      = trial.position_y_cut
        position_z_cut      = trial.position_z_cut
        rotation_x_cut      = trial.rotation_x_cut
        rotation_y_cut      = trial.rotation_y_cut
        rotation_z_cut      = trial.rotation_z_cut
        rotation_w_cut      = trial.rotation_w_cut
        rover_time_cut      = trial.rover_time_cut
        accel_x_cut         = trial.accel_x_cut
        accel_y_cut         = trial.accel_y_cut
        accel_z_cut         = trial.accel_z_cut
        gyro_x_cut          = trial.gyro_x_cut
        gyro_y_cut          = trial.gyro_y_cut
        gyro_z_cut          = trial.gyro_z_cut
        motor_rpm_1_cut     = trial.motor_rpm_1_cut
        motor_rpm_2_cut     = trial.motor_rpm_2_cut
        motor_rpm_3_cut     = trial.motor_rpm_3_cut
        motor_rpm_4_cut     = trial.motor_rpm_4_cut
        motor_current_1_cut = trial.motor_current_1_cut
        motor_current_2_cut = trial.motor_current_2_cut
        motor_current_3_cut = trial.motor_current_3_cut
        motor_current_4_cut = trial.motor_current_4_cut

        time_elapsed = rover_time_cut[-1] - rover_time_cut[0]

        # average speed (cm/s)
        # 850 = gear ratio, 60 min to sec, 2*pi*r = 2pi(0.075cm/s) = circumference. Get angular velocity in m/s
        motor_vel_1_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_1_cut] 
        motor_vel_2_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_2_cut]
        motor_vel_3_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_3_cut]
        motor_vel_4_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_4_cut]

        motor_1_avg_vel = np.mean([abs(vel) for  vel in motor_vel_1_cut if abs(vel) > 0.03])
        motor_2_avg_vel = np.mean([abs(vel) for  vel in motor_vel_2_cut if abs(vel) > 0.03])
        motor_3_avg_vel = np.mean([abs(vel) for  vel in motor_vel_4_cut if abs(vel) > 0.03])
        motor_4_avg_vel = np.mean([abs(vel) for  vel in motor_vel_4_cut if abs(vel) > 0.03])

        target_speed = np.mean([motor_1_avg_vel, motor_2_avg_vel, motor_3_avg_vel, motor_4_avg_vel])

        json_trial_params[file_name] = {"time_elapsed": time_elapsed, "angle": degrees, "target_speed": target_speed}

    json.dump(json_trial_params, open(output_file_path, "w"), indent=4)