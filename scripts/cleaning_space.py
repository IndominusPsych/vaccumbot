#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

def publish_point(x, y, z):
    # Initialize the ROS node
    rospy.init_node('point_publisher', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    
    # Create the PointStamped message
    point = PointStamped()
    
    # Set the header
    point.header = Header()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "map"  # Frame ID can be changed as per your requirement
    
    # Set the point coordinates
    point.point.x = x
    point.point.y = y
    point.point.z = z
    
    # Publish the point
    pub.publish(point)
    
    rospy.loginfo(f"Published point: ({x}, {y}, {z}) to /clicked_point")

if __name__ == '__main__':
    try:
        # Example points to publish
        points_to_publish = [(-0.9168656468391418, 2.07215881347656250), 
                                (3.7312662601470947, 1.9001405239105225), 
                                (-0.622028648853302, 3.29244065284729), 
                                (3.7280995845794678, 3.868736743927002),
                                (3.666754722595215, 1.9620170593261719),
                                (-0.9550445675849915, 2.0840022563934326)]
        
        # Publish each point
        for point in points_to_publish:
            publish_point(point[0], point[1], 0)
            rospy.sleep(1)  # Sleep for a second between publishes
        
    except rospy.ROSInterruptException:
        pass