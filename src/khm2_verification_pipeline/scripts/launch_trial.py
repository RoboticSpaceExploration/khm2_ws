#!/usr/bin/env python3

import rospy
import roslaunch

pkg = 'rosbag'
exe = 'record'
node = roslaunch.core.Node(pkg, exe, args="-o $(find khm2_verification_pipeline)/bags/sim /joint_states /front_left_torque /front_right_torque /back_left_torque /back_right_torque /imu /gazebo/model_states", output="screen")

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
rospy.loginfo("started rosbag record node")

rospy.sleep(5)

# import roslaunch

# package = 'rqt_gui'
# executable = 'rqt_gui'
# node = roslaunch.core.Node(package, executable)

# launch = roslaunch.scriptapi.ROSLaunch()
# launch.start()

# process = launch.launch(node)
# print process.is_alive()
# process.stop()