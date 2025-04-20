#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import numpy as np
from mi_robot.srv import BuscarLata, BuscarLataResponse

class BlackCanService:
    def __init__(self):
        rospy.init_node('black_can_service')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/front/image_raw', Image, self.image_callback)
        self.buscar_lata_service = rospy.Service('buscar_lata', BuscarLata, self.buscar_lata_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 30])
        self.detection_threshold_area = 100
        self.image_width = 640.0
        self.angular_speed = 0.8 # Ajusta según sea necesario
        self.centering_threshold = 0.1
        self.lata_encontrada = False
        self.centro_x_lata = -1.0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_width = float(cv_image.shape[1])
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, self.lower_black, self.upper_black)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            self.lata_encontrada = False
            self.centro_x_lata = -1.0

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.detection_threshold_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = float(M["m10"] / M["m00"])
                        position = self.calculate_position(center_x)
                        if position == 0:
                            self.lata_encontrada = True
                            self.centro_x_lata = center_x
        except Exception as e:
            rospy.logerr("Error en image_callback: %s", str(e))

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

    def buscar_lata_callback(self, req):
        rospy.loginfo("Servicio buscar_lata llamado.")
        response = BuscarLataResponse()
        response.lata_encontrada = False
        response.centro_x = -1.0
        twist = Twist()
        start_time = rospy.Time.now()
        timeout = rospy.Duration(15) # Tiempo máximo para buscar la lata

        while not rospy.is_shutdown() and not self.lata_encontrada and (rospy.Time.now() - start_time < timeout):
            if not self.lata_encontrada:
                # Intenta encontrar la lata rotando
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1) # Controla la velocidad de rotación y el tiempo entre checks
            else:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                response.lata_encontrada = True
                response.centro_x = self.centro_x_lata
                rospy.loginfo("Lata negra encontrada y centrada.")
                return response

        # Si el bucle termina sin encontrar la lata
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Tiempo de espera agotado, no se encontró la lata.")
        return response

if __name__ == '__main__':
    try:
        service = BlackCanService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass