#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import sys

# Callbacks definition

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
        
rospy.init_node('goal_pose')

navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

# Navigation goal
default_x = -1.0
default_y = 2.0
if len(sys.argv) > 1:
    current_x = float(sys.argv[-2])
    current_y = float(sys.argv[-1])

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = current_x
goal.target_pose.pose.position.y = current_y
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.0
goal.target_pose.pose.orientation.w = 1.0

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
    rospy.logerr("Action server not available!")
else:
    rospy.loginfo ( navclient.get_result())
    
