#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist

def shutdown_node(event):
    rospy.signal_shutdown("Trial complete")

def main():
    rospy.init_node('run_trial_node')
    # read trial parameters
    time = rospy.get_param('/trial_params/time', 10)
    speed = rospy.get_param('/trial_params/speed', 6)
    angle = rospy.get_param('/trial_params/angle', 0)

    cmd_vel_pub = rospy.Publisher("/robot1_velocity_controller/cmd_vel", Twist, queue_size=10)

    timer = rospy.Timer(rospy.Duration(secs=time), shutdown_node)
    rospy.loginfo("Starting trial...")
    rospy.loginfo(f"Trial duration: {time} seconds")
    rospy.loginfo(f"Rover speed: {speed} cm/s")
    rospy.loginfo(f"Angle: {angle} degrees")

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = speed / 100 # convert to m/s
        cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    main()