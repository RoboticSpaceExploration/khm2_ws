#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
import pandas as pd
import os




if __name__ == '__main__':

    filepath = "/home/roselab/Desktop/hans/control scripts/Control inputs.xlsx"
    df = pd.read_excel(filepath, engine='openpyxl')

    cmd_time = df['Time in seconds'].values.tolist()
    ls_time = [float(x) for x in cmd_time]
    cmd_lin = df['linear_vel.x'].values.tolist()
    cmd_ang = df['angular_vel.y'].values.tolist()
    ls_cmd = [[lin, ang] for lin, ang in zip(cmd_lin, cmd_ang)]

    rospy.init_node("control_rover")
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/robot1_velocity_controller/cmd_vel", Twist, queue_size = 10)

    rate = rospy.Rate(10)

    for seconds, cmd in zip(ls_time, ls_cmd):
        start = time.time()
        time.perf_counter()
        elapsed = 0
        while elapsed < seconds:
            elapsed = time.time() - start
            time.sleep(1)
            msg = Twist()
            msg.linear.x = cmd[0]
            msg.angular.z = cmd[1]
            pub.publish(msg)
            rate.sleep()
