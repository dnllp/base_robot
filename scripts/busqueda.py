#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlackCanDetector:
    def __init__(self):
        rospy.init_node('black_can_controller')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/front/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 30]) # Adjust upper bound as needed
        self.detection_threshold_area = 100 # Minimum area to consider a can
        self.image_width = 640.0 # Default, will be updated in callback (as float for calculations)
        self.angular_speed = 4.0 # Adjust for desired rotation speed
        self.centering_threshold = 0.1 # Percentage of image width to consider centered

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_width = float(cv_image.shape[1])
            # Make a copy for visualization (comment out if not needed)
            visualization_image = cv_image.copy()
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.lower_black, self.upper_black)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        twist = Twist()
        center_x = -1 # Initialize center_x

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.detection_threshold_area:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = float(M["m10"] / M["m00"])
                    position = self.calculate_position(center_x)

                    if position == 1:
                        twist.angular.z = self.angular_speed # Rotate left
                    elif position == -1:
                        twist.angular.z = -self.angular_speed # Rotate right
                    else:
                        twist.angular.z = 0.0 # Centered, stop rotation
                    # Draw the contour and center (comment out if not needed)
                    cv2.drawContours(visualization_image, [largest_contour], 0, (0, 255, 0), 2)
                    cv2.circle(visualization_image, (int(center_x), int(M["m01"] / M["m00"])), 5, (0, 0, 255), -1)
                else:
                    twist.angular.z = 0.0 # No valid center, stop rotation
            else:
                twist.angular.z = 0.0 # No can large enough, stop rotation
        else:
            twist.angular.z = 0.0 # No black can detected, stop rotation

        self.cmd_vel_pub.publish(twist)

        # Display the image with detection (comment out the entire block if not needed)
        window_name = "Black Can Detection"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        if 'visualization_image' in locals():
            cv2.imshow(window_name, visualization_image)
            cv2.waitKey(1)
        else:
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
                cv2.destroyWindow(window_name)

    def calculate_position(self, center_x):
        center_image = self.image_width / 2.0
        deviation = abs(center_x - center_image)
        tolerance = self.image_width * self.centering_threshold

        if deviation > tolerance and center_x < center_image:
            return -1 # Left
        elif deviation > tolerance and center_x > center_image:
            return 1 # Right
        else:
            return 0 # Center

if __name__ == '__main__':
    try:
        detector = BlackCanDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass