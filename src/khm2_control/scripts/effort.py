#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def set_joint_velocity():
    rospy.init_node('set_joint_velocity_node')

    # Publisher to /gazebo/set_model_state
    pub = rospy.Publisher('/gazebo/set_joint_state', JointState, queue_size=10)
    
    joint_name = "FR_TO_BASELINK"  # replace with your joint's name
    target_velocity = 2.0  # desired velocity in radians/second
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.name = [joint_name]
        joint_state.velocity = [target_velocity]
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    set_joint_velocity()
