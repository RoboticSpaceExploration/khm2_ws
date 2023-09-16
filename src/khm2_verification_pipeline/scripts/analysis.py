#!/usr/bin/env python3
<<<<<<< HEAD
import pickle
import matplotlib.pyplot as plt
import math
import os
import sys
=======
import rospkg
import pickle
import os
>>>>>>> d6e6974418bc85c990dd74811f846e600c1a50ca
import re # regex for parsing file names
from typing import Dict

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

def get_cadre_data(file, path):
    with open(path + "/" + f"{file}", "rb") as f:
        return pickle.load(f)
        
<<<<<<< HEAD
    # copy following:

    # vicon_time_cut = mydata.vicon_time_cut
    # position_x_cut = mydata.position_x_cut
    # position_y_cut = mydata.position_y_cut
    # position_z_cut = mydata.position_z_cut
    # rotation_x_cut = mydata.rotation_x_cut
    # rotation_y_cut = mydata.rotation_y_cut
    # rotation_z_cut = mydata.rotation_z_cut
    # rotation_w_cut = mydata.rotation_w_cut
    # rover_time_cut = mydata.rover_time_cut
    # accel_x_cut = mydata.accel_x_cut
    # accel_y_cut = mydata.accel_y_cut
    # accel_z_cut = mydata.accel_z_cut
    # gyro_x_cut = mydata.gyro_x_cut
    # gyro_y_cut = mydata.gyro_y_cut
    # gyro_z_cut = mydata.gyro_z_cut
    # motor_rpm_1_cut = mydata.motor_rpm_1_cut
    # motor_rpm_2_cut = mydata.motor_rpm_2_cut
    # motor_rpm_3_cut = mydata.motor_rpm_3_cut
    # motor_rpm_4_cut = mydata.motor_rpm_4_cut
    # motor_current_1_cut = mydata.motor_current_1_cut
    # motor_current_2_cut = mydata.motor_current_2_cut
    # motor_current_3_cut = mydata.motor_current_3_cut
    # motor_current_4_cut = mydata.motor_current_4_cut

    # 850 = gear ratio, 60 min to sec, 2*pi*r = 2pi(0.075cm/s) = circumference. Get angular velocity in m/s
    # motor_vel_1_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_1_cut] 
    # motor_vel_2_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_2_cut]
    # motor_vel_3_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_3_cut]
    # motor_vel_4_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_4_cut]

    # test plots
    # fig = plt.figure(figsize=(40,20))
    # ax1 = fig.add_subplot(3,1,1)
    # ax1.plot(rover_time_cut, position_x_cut, label= 'x')
    # plt.xlabel('time [sec]')
    # plt.ylabel('x position [m]')

    # ax2 = fig.add_subplot(3,1,2)
    # ax2.plot(rover_time_cut, position_y_cut, label= 'y')
    # plt.xlabel('time [sec]')
    # plt.ylabel('y position [m]')

    # ax3 = fig.add_subplot(3,1,3)
    # ax3.plot(rover_time_cut, position_z_cut, label= 'y')
    # plt.xlabel('time [sec]')
    # plt.ylabel('z position [m]')

    # ax4 = fig.add_subplot(3,1,3)
    # ax4.plot(rover_time_cut, motor_rpm_1_cut, label= 'y')
    # plt.xlabel('time [sec]')
    # plt.ylabel('motor rpm [rotations/min]')

    # fig.suptitle(f'position cut for {file}')
    # # plt.show()

    # find all the files and sort them by inclination
=======
>>>>>>> d6e6974418bc85c990dd74811f846e600c1a50ca

# access each file by degrees with 
# deg0 = [file for file in files.keys() if files[file] == 0]
def get_cadre_files() -> (Dict[str, data], Dict[str, int]):
<<<<<<< HEAD
    path = "/mnt/d/School/College/Rose/verification_pipeline/cut_data"
=======
    path = os.path.join(rospkg.RosPack().get_path('khm2_verification_pipeline'), 'data/cut_data')
>>>>>>> d6e6974418bc85c990dd74811f846e600c1a50ca
    files = {}
    for file_name in os.listdir(path):
        if file_name.endswith(".p"):
            regex = r"(\d+)DEG" # captures the number before DEG
            degrees = int(re.findall(regex, file_name)[0]) # take the first group
            files[file_name] = degrees
    files = dict(sorted(files.items(), key=lambda item: item[1])) # sort them

    data = {}
    for file in files.keys():
        mydata = get_cadre_data(file, path)
        data[file] = mydata
    files["path"] = path
    return (data, files)
    
<<<<<<< HEAD
=======

# copy following template to test using the data:

# vicon_time_cut = mydata.vicon_time_cut
# position_x_cut = mydata.position_x_cut
# position_y_cut = mydata.position_y_cut
# position_z_cut = mydata.position_z_cut
# rotation_x_cut = mydata.rotation_x_cut
# rotation_y_cut = mydata.rotation_y_cut
# rotation_z_cut = mydata.rotation_z_cut
# rotation_w_cut = mydata.rotation_w_cut
# rover_time_cut = mydata.rover_time_cut
# accel_x_cut = mydata.accel_x_cut
# accel_y_cut = mydata.accel_y_cut
# accel_z_cut = mydata.accel_z_cut
# gyro_x_cut = mydata.gyro_x_cut
# gyro_y_cut = mydata.gyro_y_cut
# gyro_z_cut = mydata.gyro_z_cut
# motor_rpm_1_cut = mydata.motor_rpm_1_cut
# motor_rpm_2_cut = mydata.motor_rpm_2_cut
# motor_rpm_3_cut = mydata.motor_rpm_3_cut
# motor_rpm_4_cut = mydata.motor_rpm_4_cut
# motor_current_1_cut = mydata.motor_current_1_cut
# motor_current_2_cut = mydata.motor_current_2_cut
# motor_current_3_cut = mydata.motor_current_3_cut
# motor_current_4_cut = mydata.motor_current_4_cut

# 850 = gear ratio, 60 min to sec, 2*pi*r = 2pi(0.075cm/s) = circumference. Get angular velocity in m/s
# motor_vel_1_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_1_cut] 
# motor_vel_2_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_2_cut]
# motor_vel_3_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_3_cut]
# motor_vel_4_cut = [(rpm / 850 / 60) * 2 * math.pi * 0.075 for rpm in motor_rpm_4_cut]

# test plots
# fig = plt.figure(figsize=(40,20))
# ax1 = fig.add_subplot(3,1,1)
# ax1.plot(rover_time_cut, position_x_cut, label= 'x')
# plt.xlabel('time [sec]')
# plt.ylabel('x position [m]')

# ax2 = fig.add_subplot(3,1,2)
# ax2.plot(rover_time_cut, position_y_cut, label= 'y')
# plt.xlabel('time [sec]')
# plt.ylabel('y position [m]')

# ax3 = fig.add_subplot(3,1,3)
# ax3.plot(rover_time_cut, position_z_cut, label= 'y')
# plt.xlabel('time [sec]')
# plt.ylabel('z position [m]')

# ax4 = fig.add_subplot(3,1,3)
# ax4.plot(rover_time_cut, motor_rpm_1_cut, label= 'y')
# plt.xlabel('time [sec]')
# plt.ylabel('motor rpm [rotations/min]')

# fig.suptitle(f'position cut for {file}')
# # plt.show()

# find all the files and sort them by inclination
>>>>>>> d6e6974418bc85c990dd74811f846e600c1a50ca
