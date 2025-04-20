#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedDepositController:
    def __init__(self):
        rospy.init_node('red_deposit_controller')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rear/image_raw', Image, self.image_callback)
        self.mode_sub = rospy.Subscriber('/modo', Int8, self.mode_callback)
        self.distance_sub = rospy.Subscriber('/distancia', Float32, self.distance_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.descarga_pub = rospy.Publisher('/descarga', Bool, queue_size=1)

        self.lower_red_lower = np.array([0, 100, 100])
        self.upper_red_lower = np.array([10, 255, 255])
        self.lower_red_upper = np.array([170, 100, 100])
        self.upper_red_upper = np.array([180, 255, 255])

        self.detection_threshold_area = 100 # Minimum area for the deposit
        self.image_width = 640.0
        self.angular_speed = 2.5
        self.linear_speed_forward = 1.1
        self.linear_speed_backward = -1.1
        self.centering_threshold = 0.1
        self.target_distance = 0.10 # 10 cm
        self.current_distance = None
        self.current_mode = 0
        self.deposit_centered = False
        self.approaching = False

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode == 3:
            rospy.loginfo("Red deposit controller activated.")
            self.deposit_centered = False
            self.approaching = False

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def image_callback(self, msg):
        if self.current_mode != 3:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_width = float(cv_image.shape[1])
            # Make a copy for visualization (comment out if not needed)
            visualization_image = cv_image.copy()
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_lower = cv2.inRange(hsv_image, self.lower_red_lower, self.upper_red_lower)
        mask_upper = cv2.inRange(hsv_image, self.lower_red_upper, self.upper_red_upper)
        mask = cv2.bitwise_or(mask_lower, mask_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        twist = Twist()
        descarga_msg = Bool()
        descarga_msg.data = False
        center_x = -1

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.detection_threshold_area:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = float(M["m10"] / M["m00"])
                    position = self.calculate_position(center_x)

                    if not self.deposit_centered:
                        if position == 1:
                            twist.angular.z = self.angular_speed
                            twist.linear.x = 0.0
                        elif position == -1:
                            twist.angular.z = -self.angular_speed
                            twist.linear.x = 0.0
                        else:
                            twist.angular.z = 0.0
                            twist.linear.x = 0.0
                            self.deposit_centered = True
                            rospy.loginfo("Red deposit centered.")
                            self.approaching = True
                    elif self.approaching:
                        if self.current_distance is not None:
                            if self.current_distance > self.target_distance:
                                twist.linear.x = self.linear_speed_backward
                                twist.angular.z = 0.0
                            else:
                                twist.linear.x = 0.0
                                twist.angular.z = 0.0
                                descarga_msg.data = True
                                rospy.loginfo("Reached deposit. Initiating descarga.")
                                self.descarga_pub.publish(descarga_msg)
                                #rospy.sleep(5) # Simulate descarga
                                mode_msg = Int8()
                                mode_msg.data = 2
                                rospy.Publisher('/modo', Int8, queue_size=1).publish(mode_msg)
                                rospy.loginfo("Descarga complete. Setting mode to 2.")
                                self.current_mode = 2
                                self.deposit_centered = False
                                self.approaching = False
                                
                        else:
                            rospy.logwarn_once("Waiting for distance data on /distancia")
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                    # Draw the contour and center (comment out if not needed)
                    cv2.drawContours(visualization_image, [largest_contour], 0, (0, 255, 0), 2)
                    if center_x != -1 and "m01" in M and M["m00"] != 0:
                        center_y = int(M["m01"] / M["m00"])
                        cv2.circle(visualization_image, (int(center_x), center_y), 5, (255, 0, 0), -1)
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.descarga_pub.publish(descarga_msg)

        # Display the image with detection (comment out the entire block if not needed)
        window_name = "Red Deposit Detection"
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
        detector = RedDepositController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass