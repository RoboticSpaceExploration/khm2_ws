#!/usr/bin/env python3

import rospy
import rospkg
import roslaunch
import xml.etree.ElementTree as ET # GO HOME
import os
import pickle
import json
import re
from preprocess_trial_params import generate_trial_params_json, data # data class needed for pickle

rospy.init_node("init_trial_node")
rospack = rospkg.RosPack()

# create trial params json file if it doesn't exist
data_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data')
if not os.path.isdir(data_path):
    os.mkdir(data_path)
if not os.path.isfile(os.path.join(data_path, 'trial_params.json')):
    generate_trial_params_json()

# load trial params from json
with open(os.path.join(data_path, 'trial_params.json'), 'r') as file:
    trial_params = json.load(file)

# get cadre file to use
# look in data/cut_data folder for all the cadre pickle files
cadre_data_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data/cut_data')
cadre_files = None
try:
    cadre_files = os.listdir(cadre_data_path)
except:
    rospy.logerr(f"Tried looking for {cadre_data_path}, but it doesn't exist. Make sure the cadre data exists in that folder.")
    exit()

# print out the cadre files
def pretty_print_list(l):
    l.sort()
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
cadre_files = os.listdir(cadre_data_path)
cadre_files = [file for file in cadre_files if file.endswith(".p") and 'refined' in file]
pretty_print_list(cadre_files)

# get trial to use
cadre_file_name = input("Copy paste cadre data trial file above to use for this trial.\n> ")
print(os.path.join(cadre_data_path, cadre_file_name))
with open(os.path.join(cadre_data_path, cadre_file_name), 'rb') as file:
    if pickle.load(file) == None:
        rospy.logerr(f"{os.path.join(cadre_data_path, cadre_file_name)} either doesn't exist or is corrupted. Stopping trial")
        exit()
rospy.set_param("/current_trial_file_name", cadre_file_name)

# =================================

# set up the world launch file's rotation
world_file_path = os.path.join(rospack.get_path('khm2_gazebo'), 'worlds/verification_pipeline_debug.world')
world_file_path = os.path.join(rospack.get_path('khm2_gazebo'), 'worlds/soft_soil.world')
tree = ET.parse(world_file_path)
xml_root = tree.getroot()

degrees = trial_params[cadre_file_name]['angle']
radians = degrees * 3.141592 / 180
rospy.loginfo(f"Setting floor rotation to: {degrees} degrees")
floor_angle_tag = f"./world/state/model[@name='Floor']/link[@name='link_0']/pose"
floor_angle_tag = f"./world/plugin[@name='hina_ssi_physics']/sandbox/angle"
xml_root.find(floor_angle_tag).text = f"{-radians} 0 0"
tree.write(world_file_path)

# rotate the rover
rover_launch_file = os.path.join(rospack.get_path('khm2_bringup'), 'launch/rover.launch')
with open(rover_launch_file, 'r') as file:
    rover_launch_file_text = file.read()
    replacement = f'<arg name="rp" value="{radians}"/>'
    pattern = re.compile(r'<arg name="rp" value=".*"/>')
    rover_launch_file_text = re.sub(pattern, replacement, rover_launch_file_text)

# =================================

# launch gazebo and spawn rover
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

sim_args = ['khm2_bringup', 'simulation.launch']
sim_launch = roslaunch.rlutil.resolve_launch_arguments(sim_args)[0]

rover_args = ['khm2_bringup', 'rover.launch']
rover_launch = roslaunch.rlutil.resolve_launch_arguments(rover_args)[0]

launch_files = [sim_launch, rover_launch]
parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
parent.spin()
