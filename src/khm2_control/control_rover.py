#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

#ls_cmd = [[1,2], [1,2], [1,2]]
#ls_time = [1, 2, 3]

if __name__ == '__main__':
    rospy.init_node("control_rover")
    rospy.loginfo("Node has been started.")

    pub = rospy.Publisher("/robot1_velocity_controller/cmd_vel", Twist, queue_size = 10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 0.04
        msg.angular.z = 0.01
        pub.publish(msg)
        rate.sleep()
