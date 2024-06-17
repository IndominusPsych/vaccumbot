#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/overlayed_image", Image, queue_size=10)
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Convert image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        
        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                   param1=50, param2=30, minRadius=10, maxRadius=50)
        
        if circles is not None:
            circles = circles[0]  # Extract circles from the result
            self.tracked_objects = []  # Clear previous tracked objects
            
            for circle in circles:
                x, y, radius = int(circle[0]), int(circle[1]), int(circle[2])
                self.tracked_objects.append((x, y, radius))
                
                # Draw green circle around the detected object
                cv2.circle(cv_image, (x, y), radius, (0, 255, 0), 2)
                rospy.loginfo(f"Object detected at x: {x}, y: {y}, radius: {radius}")

        # Publish the processed image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Object Tracking", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('object_tracker', anonymous=True)
    ObjectTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()