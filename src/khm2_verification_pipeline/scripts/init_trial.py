#!/usr/bin/env python3

import rospy
import rospkg
import xml.etree.ElementTree as ET # GO HOME

# This python script meant to be run before gazebo is launched. Edits the world file

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