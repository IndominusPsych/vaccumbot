#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class ObstacleController:
    def __init__(self, name, initial_x, target_x, step_size):
        self.name = name
        self.initial_x = initial_x
        self.target_x = target_x
        self.step_size = step_size
        self.direction = 1  # 1 for forward, -1 for backward
        self.state_msg = ModelState()
        self.state_msg.model_name = name
        self.state_msg.reference_frame = 'world'

    def move_obstacle(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Move the obstacle
            self.state_msg.pose.position.x += self.direction * self.step_size

            # If the obstacle reaches the target distance, change direction
            if abs(self.state_msg.pose.position.x - self.initial_x) >= self.target_x:
                self.direction *= -1

            try:
                set_state(self.state_msg)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

            rate.sleep()

def set_state_client():
    rospy.wait_for_service('/gazebo/set_model_state')
    return rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_controller', anonymous=True)
    set_state = set_state_client()

    # Define parameters for each obstacle: name, initial_x, target_x, step_size
    obstacles = [
        ObstacleController('obstacle1', initial_x=0.0, target_x=4.0, step_size=0.1),
        ObstacleController('obstacle2', initial_x=0.0, target_x=3.0, step_size=0.2),
        # Add more obstacles as needed
    ]

    # Start the movement for each obstacle
    for obstacle in obstacles:
        rospy.loginfo("Starting movement for obstacle: %s" % obstacle.name)
        obstacle.move_obstacle()
