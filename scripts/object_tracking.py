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
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        fgmask = self.fgbg.apply(cv_image)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter out small contours
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                rospy.loginfo(f"Object detected at x: {x}, y: {y}, width: {w}, height: {h}")

        # Publish the processed image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Object Tracking", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('object_tracker', anonymous=True)
    ot = ObjectTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()