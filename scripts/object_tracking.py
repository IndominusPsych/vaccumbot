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
        
        # Define a simple object color range (in HSV)
        self.lower_color = (0, 0, 0)   # Adjust this range based on your object
        self.upper_color = (255, 255, 255)  # Adjust this range based on your object

        # Initialize background subtractor
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2()

        # Initialize variables to store previous object positions
        self.prev_positions = []
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Apply background subtractor to get the foreground mask
        fg_mask = self.bg_subtractor.apply(cv_image)

        # Apply some morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the foreground mask
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        # Loop through the contours to detect the object based on color
        for contour in contours:
            if cv2.contourArea(contour) < 500:
                # Ignore small contours that are likely noise
                continue

            x, y, w, h = cv2.boundingRect(contour)
            roi = cv_image[y:y+h, x:x+w]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, self.lower_color, self.upper_color)

            if cv2.countNonZero(mask) > 0:
                detected_objects = []

        # Apply additional filtering based on movement continuity
        filtered_objects = self.filter_objects_by_movement(detected_objects)

        # Draw bounding boxes around detected objects
        for (x, y, w, h) in filtered_objects:
            cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Update previous positions
        self.prev_positions = filtered_objects

        # Publish the processed image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Object Tracking", cv_image)
        cv2.waitKey(1)
    
    def filter_objects_by_movement(self, detected_objects):
        # Filter objects based on their movement continuity
        if not self.prev_positions:
            return detected_objects

        filtered_objects = []
        for (x, y, w, h) in detected_objects:
            for (px, py, pw, ph) in self.prev_positions:
                if abs(x - px) < 50 and abs(y - py) < 50:
                    filtered_objects.append((x, y, w, h))
                    break

        return filtered_objects
    
def main():
    rospy.init_node('object_tracker', anonymous=True)
    ObjectTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()