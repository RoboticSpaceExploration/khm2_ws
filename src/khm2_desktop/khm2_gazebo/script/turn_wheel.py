#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def turn_wheel_gazebo():
    rospy.init_node('turn_wheel_gazebo_node', anonymous=True)

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Define the model state message
        model_state = ModelState()
        model_state.model_name = 'single_wheel_robot'  # Change this to your wheel's model name
        model_state.reference_frame = 'world'  # or the specific frame you're interested in

        # Set only angular velocity to turn the wheel
        model_state.twist.angular.x = 0.001  # Rotational velocity; adjust as necessary
    
        
        # Leave the pose information unchanged to keep the wheel in its current position

        # Send the command
        resp = set_state(model_state)

        rospy.loginfo("SetModelState response: %s" % str(resp))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        turn_wheel_gazebo()
    except rospy.ROSInterruptException:
        pass

