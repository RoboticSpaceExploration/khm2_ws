#!/usr/bin/env python3
import pandas as pd
import pickle
import matplotlib.pyplot as plt



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
        
path = "/home/roselab/Desktop/cut_data"
file = "25DEG1_cut7_refined.p"
with open(path + "/" + f"{file}", "rb") as f:
    mydata = pickle.load(f)
    
vicon_time_cut = mydata.vicon_time_cut
position_x_cut = mydata.position_x_cut
position_y_cut = mydata.position_y_cut
position_z_cut = mydata.position_z_cut
rotation_x_cut = mydata.rotation_x_cut
rotation_y_cut = mydata.rotation_y_cut
rotation_z_cut = mydata.rotation_z_cut
rotation_w_cut = mydata.rotation_w_cut
rover_time_cut = mydata.rover_time_cut
accel_x_cut = mydata.accel_x_cut
accel_y_cut = mydata.accel_y_cut
accel_z_cut = mydata.accel_z_cut
gyro_x_cut = mydata.gyro_x_cut
gyro_y_cut = mydata.gyro_y_cut
gyro_z_cut = mydata.gyro_z_cut
motor_rpm_1_cut = mydata.motor_rpm_1_cut
motor_rpm_2_cut = mydata.motor_rpm_2_cut
motor_rpm_3_cut = mydata.motor_rpm_3_cut
motor_rpm_4_cut = mydata.motor_rpm_4_cut
motor_current_1_cut = mydata.motor_current_1_cut
motor_current_2_cut = mydata.motor_current_2_cut
motor_current_3_cut = mydata.motor_current_3_cut
motor_current_4_cut = mydata.motor_current_4_cut

fig = plt.figure(figsize=(40,20))
ax1 = fig.add_subplot(3,1,1)
ax1.plot(rover_time_cut, position_x_cut, label= 'x')
plt.xlabel('time [sec]')
plt.ylabel('x position [m]')
ax2 = fig.add_subplot(3,1,2)
ax2.plot(rover_time_cut, position_y_cut, label= 'y')
plt.xlabel('time [sec]')
plt.ylabel('y position [m]')
ax3 = fig.add_subplot(3,1,3)
ax3.plot(rover_time_cut, position_z_cut, label= 'y')
plt.xlabel('time [sec]')
plt.ylabel('z position [m]')
fig.suptitle(f'position cut for {file}')
plt.show()

