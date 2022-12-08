#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
import pandas as pd
import os
import sys



if __name__ == '__main__':

    filepath = "/home/roselab/Desktop/hans/control scripts/Trials2.xlsx"
    df = pd.read_excel(filepath, sheet_name="basic", engine='openpyxl')
    #current_number = str(sys.argv[1])
    current_number = '4'
    current = df[current_number]
    ls_time = [int(current[15 + i * 3]) for i in range(11)]
    cmd_lin = [float(current[13 + i * 3]) for i in range(11)]
    cmd_ang = [float(current[14 + i * 3]) for i in range(11)]
    ls_cmd = [[lin, ang] for lin, ang in zip(cmd_lin, cmd_ang)]

    rospy.init_node("control_rover")
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/robot1_velocity_controller/cmd_vel", Twist, queue_size = 10)

    rate = rospy.Rate(10)

    for seconds, cmd in zip(ls_time, ls_cmd):
        start = time.time()
        timeout = start + seconds
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = cmd[0]
            msg.angular.z = cmd[1]
            pub.publish(msg)
            rate.sleep()
            if time.time() > timeout:
                break
