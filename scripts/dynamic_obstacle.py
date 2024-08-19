#!/usr/bin/env python3

import rospy
import threading
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelState, SetModelState

class ObstacleController:
    def __init__(self, name, target_x, target_y, step_size):
        self.name = name
        self.target_x = target_x
        self.target_y = target_y
        self.step_size = step_size
        self.state_msg = ModelState()
        self.state_msg.model_name = name
        self.state_msg.reference_frame = 'world'
        self.get_initial_position()
        self.moving_to_target = True

    def get_initial_position(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = get_state(self.name, 'world')
            self.initial_x = resp.pose.position.x
            self.initial_y = resp.pose.position.y
            self.state_msg.pose.position.x = self.initial_x
            self.state_msg.pose.position.y = self.initial_y
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def move_obstacle(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.moving_to_target:
                # Move towards the target
                self.state_msg.pose.position.x += self.step_size if self.state_msg.pose.position.x < self.target_x else 0
                self.state_msg.pose.position.y += self.step_size if self.state_msg.pose.position.y < self.target_y else 0
                
                # Check if target is reached
                if (self.state_msg.pose.position.x >= self.target_x and 
                    self.state_msg.pose.position.y >= self.target_y):
                    self.moving_to_target = False
            else:
                # Move back to the initial position
                self.state_msg.pose.position.x -= self.step_size if self.state_msg.pose.position.x > self.initial_x else 0
                self.state_msg.pose.position.y -= self.step_size if self.state_msg.pose.position.y > self.initial_y else 0
                
                # Check if initial position is reached
                if (self.state_msg.pose.position.x <= self.initial_x and 
                    self.state_msg.pose.position.y <= self.initial_y):
                    self.moving_to_target = True

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

    # Define parameters for each obstacle: name, target_x, target_y, step_size
    obstacles = [
        ObstacleController('obstacle1', target_x=2.0, target_y=0.0, step_size=0.02),
        ObstacleController('obstacle2', target_x=-3.0, target_y=3.0, step_size=0.02),
        # Add more obstacles as needed
    ]

    # Start the movement for each obstacle in a separate thread
    threads = []
    for obstacle in obstacles:
        rospy.loginfo("Starting movement for obstacle: %s" % obstacle.name)
        t = threading.Thread(target=obstacle.move_obstacle)
        t.start()
        threads.append(t)

    # Join threads to ensure the script doesn't exit until all threads are complete
    for t in threads:
        t.join()
