#!/usr/bin/env python3

import rospy
import rospkg
import roslaunch
import xml.etree.ElementTree as ET # GO HOME
import os
import pickle

# set up the world launch file's rotation
rospy.init_node("set_floor_rotation_node")
rospack = rospkg.RosPack()
world_file_path = rospack.get_path('khm2_gazebo') + '/worlds/verification_pipeline_debug.world'
tree = ET.parse(world_file_path)
xml_root = tree.getroot()

degrees = rospy.get_param("/trial_params/angle", 0)
radians = degrees * 3.141592 / 180
rospy.loginfo(f"Setting floor rotation to: {degrees} degrees")
floor_link = f"./world/state/model[@name='Floor']/link[@name='link_0']/pose"
xml_root.find(floor_link).text = f"-0.010923 0.300375 -0.916015 0 {-radians} 0"
tree.write(world_file_path)

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

# get cadre file to use
# look in ../data/cut_data folder for all the cadre pickle files
cadre_files_path = rospack.get_path('khm2_verification_pipeline') + '/data/cut_data'
cadre_files = None
# try:
#     cadre_files = os.listdir(cadre_data_path)
# except:
#     rospy.logerr(f"Tried looking for {cadre_data_path}, but it doesn't exist. Make sure the cadre data exists in that folder.")
#     exit()
def pretty_print_list(l):
    col_width = max(len(word) for word in l) + 2  # padding
    max_per_col = 3
    i = 0
    for file in l:
        if i == max_per_col:
            print()
            i = 0
        i += 1
        print(f"{file:{col_width}}", end="")
    print("\n")
cadre_files = os.listdir(cadre_files_path)
cadre_files = [file for file in cadre_files if file.endswith(".p") and 'refined' in file]
pretty_print_list(cadre_files)
cadre_file_name = input("Choose cadre data trial above to use for this trial.")
if pickle.load(os.path.join(cadre_files_path + cadre_file_name)) == None:
    rospy.logerr(f"{os.path.join(cadre_files_path, cadre_file_name)} either doesn't exist or is corrupted.")

rospy.set_param("/trial_params/cadre_trial_file", cadre_file_name)

# parse pickle file for params

rospy.logerr(f"Tried looking for {cadre_files_path}, but it doesn't exist. Make sure the cadre data exists in that folder.")
exit()

# launch gazebo and spawn rover
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# trial_params_args = ['khm2_verification_pipeline', 'trial_params.launch']
# trial_params_launch = roslaunch.rlutil.resolve_launch_arguments(trial_params_args)[0]
# print(trial_params_launch)

sim_args = ['khm2_bringup', 'simulation.launch']
sim_launch = roslaunch.rlutil.resolve_launch_arguments(sim_args)[0]

rover_args = ['khm2_bringup', 'rover.launch']
rover_launch = roslaunch.rlutil.resolve_launch_arguments(rover_args)[0]

launch_files = [sim_launch, rover_launch]
parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
parent.spin()
