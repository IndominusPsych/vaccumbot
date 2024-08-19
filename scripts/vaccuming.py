#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def control_joint():
    rospy.init_node('fvc_controller')
    pub = rospy.Publisher('/fvc_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    velocity = Float64()
    velocity.data = 30.0  # desired velocity in radians per second

    while not rospy.is_shutdown():
        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_joint()
    except rospy.ROSInterruptException:
        pass