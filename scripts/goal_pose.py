#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def send_goal(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0

    goal.pose.orientation.z = yaw

    goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav')

        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        rospy.sleep(2)  # Wait for publisher to initialize

        # Define your waypoints here
        waypoints = [
            (1.0, 1.0, 0.0),
            (2.0, 2.0, 0.0),
            (-1.0, -1.0, 0.0)
        ]

        for waypoint in waypoints:
            send_goal(waypoint[0], waypoint[1], waypoint[2])
            rospy.sleep(10)  # Wait for some time before sending the next goal

    except rospy.ROSInterruptException:
        pass