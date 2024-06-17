#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class VaccumbotNavigator:
    def __init__(self):
        rospy.init_node('vaccumbot_navigator', anonymous=True)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    def navigate_to_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        rospy.loginfo("Sending goal...")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
            return True
        else:
            rospy.loginfo("Failed to reach goal")
            return False

if __name__ == '__main__':
    try:
        navigator = VaccumbotNavigator()

        # Define the goal pose (position and orientation)
        goals = [[1, 2, 0, 0],[2, 3, 0, 0]]
        for goal in goals:
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.orientation.z = goal[2]
            goal_pose.pose.orientation.w = goal[3]
            feedback = navigator.navigate_to_goal(goal_pose.pose)
            if feedback == True:
                continue
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")
