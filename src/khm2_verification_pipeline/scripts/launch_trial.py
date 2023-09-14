#!/usr/bin/env python3
import rospy
import rospkg
import roslaunch
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import json
import os

# GLOBAL STATE
rospack = rospkg.RosPack()
# load trial params from json
data_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data/trial_params.json')
data_dir_path = os.path.join(rospack.get_path('khm2_verification_pipeline'), 'data')
with open(data_path, 'r') as json_file:
    trial_params = json.load(json_file)
# read trial parameters
cadre_file_name = rospy.get_param('/current_trial_file_name', 'no_cadre_file_name')
time = trial_params[cadre_file_name]["time_elapsed"] # s
speed = trial_params[cadre_file_name]["target_speed"] # m/s
angle = trial_params[cadre_file_name]["angle"] # degrees


already_reached_steady_state = False
steady_state_time = rospy.Publisher("/steady_state_time", Float32, queue_size=10, latch=True)
joint_state_time = 0
done_running_trial = False

# =================== START ROSLAUNCH SETUP =================
file_name = f"sim_{angle}DEG"
# launch rosbag record and close when node is done with trial
rosbag_pkg = 'rosbag'
rosbag_exe = 'record'
rosbag_node = roslaunch.core.Node(rosbag_pkg, rosbag_exe, args=f"-o $(find khm2_verification_pipeline)/data/{file_name} /joint_states /front_left_torque /front_right_torque /back_left_torque /back_right_torque /imu /gazebo/model_states /steady_state_time", output="screen")

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

rosbag_process = launch.launch(rosbag_node)
rospy.loginfo("started rosbag record node")
# =================== END ROSLAUNCH SETUP ===============

def shutdown_node(event):
    global done_running_trial
    done_running_trial = True

def steady_state_timer_callback(timerEvent):
    steady_state_time.publish(joint_state_time)

def joint_states_callback(msg):
    global already_reached_steady_state
    joint_state_time = msg.header.stamp.to_sec()
    if not already_reached_steady_state:
        if msg.velocity[0] > speed * 0.95 and msg.velocity[1] > speed * 0.95 and msg.velocity[2] > speed * 0.95 and msg.velocity[3] > speed * 0.95:
            rospy.loginfo("Reached steady state")
            rospy.Timer(rospy.Duration(secs=time), shutdown_node) # start timer from steady state
            already_reached_steady_state = True
            rospy.Timer(rospy.Duration(secs=0.3), steady_state_timer_callback, oneshot=True)

if __name__ == "__main__":
    rospy.init_node('run_trial_node')

    joint_states_sub = rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    cmd_vel_pub = rospy.Publisher("/robot1_velocity_controller/cmd_vel", Twist, queue_size=100)

    rospy.loginfo("Starting trial...")
    rospy.loginfo(f"Trial duration: {time} seconds")
    rospy.loginfo(f"Rover speed: {speed} cm/s")
    rospy.loginfo(f"Angle: {angle} degrees")

    rate = rospy.Rate(1000)
    while not done_running_trial:
        msg = Twist()
        msg.linear.x = speed # m/s
        cmd_vel_pub.publish(msg)
        rate.sleep()

    # rosbag record automatically closes when script finishes executing
    # record rosbag file to cadre trial mapping
    print(os.path.join(data_dir_path, 'sim_trial_to_cadre_trial.json'))
    if not os.path.isfile(os.path.join(data_dir_path, 'sim_trial_to_cadre_trial.json')):
        sim_trial_to_cadre_trial = {}
    else:
        with open(os.path.join(data_dir_path, 'sim_trial_to_cadre_trial.json'), 'r') as json_file:
            sim_trial_to_cadre_trial = json.load(json_file)
    # look for rosbag 
    for file in os.listdir(data_dir_path):
        if file.endswith('.active'):
            sim_trial_to_cadre_trial[file[:-7]] = cadre_file_name
            json.dump(sim_trial_to_cadre_trial, open(os.path.join(data_dir_path, 'sim_trial_to_cadre_trial.json'), 'w'), indent=4)
            rospy.loginfo(f"Recorded sim trial {file[:-7]} to cadre trial {cadre_file_name}")
            break