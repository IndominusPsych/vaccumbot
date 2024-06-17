#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time

def move_obstacle():
    rospy.init_node('dynamic_obstacle_controller', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state_msg = ModelState()

    state_msg.model_name = 'obstacle1'
    state_msg.reference_frame = 'default'

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        state_msg.pose.position.x += 0.1
        state_msg.pose.position.y += 0.1
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1.0

        try:
            set_state(state_msg)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_obstacle()
    except rospy.ROSInterruptException:
        pass
