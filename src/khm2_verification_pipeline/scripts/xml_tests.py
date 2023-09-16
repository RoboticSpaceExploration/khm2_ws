#!/usr/bin/env python3

import rospy
import rospkg
import xml.etree.ElementTree as ET # GO HOME

# This python script meant to be run before gazebo is launched. Edits the world file
num = input("Enter the number of degrees to rotate the floor: ")
print(num)

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

