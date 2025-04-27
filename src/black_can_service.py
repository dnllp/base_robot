#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import numpy as np
from base_robot.srv import BuscarLata, BuscarLataResponse

class ImprovedBlackCanService:
    def __init__(self):
        rospy.init_node('improved_black_can_service')
        self.bridge = CvBridge()
        
        # Cargar parámetros configurables
        self.load_parameters()
        
        self.image_sub = rospy.Subscriber('/camera/front/image_raw', Image, self.image_callback)
        self.buscar_lata_service = rospy.Service('buscar_lata', BuscarLata, self.buscar_lata_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.lata_encontrada = False
        self.centro_x_lata = -1.0
        self.last_detection_time = rospy.Time(0)
        self.search_pattern_step = 0

    def load_parameters(self):
        """Carga parámetros configurables desde el servidor de parámetros"""
        self.lower_black = np.array([
            rospy.get_param('~lower_black_h', 0),
            rospy.get_param('~lower_black_s', 0),
            rospy.get_param('~lower_black_v', 0)
        ])
        self.upper_black = np.array([
            rospy.get_param('~upper_black_h', 180),
            rospy.get_param('~upper_black_s', 255),
            rospy.get_param('~upper_black_v', 30)
        ])
        self.detection_threshold_area = rospy.get_param('~detection_threshold_area', 100)
        self.angular_speed = rospy.get_param('~angular_speed', 0.8)
        self.centering_threshold = rospy.get_param('~centering_threshold', 0.1)
        self.detection_timeout = rospy.get_param('~detection_timeout', 1.0)  # segundos

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image_width = float(cv_image.shape[1])
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Mejorar detección con filtrado morfológico
            mask = cv2.inRange(hsv_image, self.lower_black, self.upper_black)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.detection_threshold_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = float(M["m10"] / M["m00"])
                        position = self.calculate_position(center_x, image_width)
                        if position == 0:  # Centrado
                            self.lata_encontrada = True
                            self.centro_x_lata = center_x
                            self.last_detection_time = rospy.Time.now()
                            return
            
            # Solo resetear si no hemos visto la lata por un tiempo
            if (rospy.Time.now() - self.last_detection_time).to_sec() > self.detection_timeout:
                self.lata_encontrada = False
                self.centro_x_lata = -1.0

        except Exception as e:
            rospy.logerr("Error en image_callback: %s", str(e))

    def calculate_position(self, center_x, image_width):
        center_image = image_width / 2.0
        deviation = abs(center_x - center_image)
        tolerance = image_width * self.centering_threshold

        if deviation > tolerance:
            return -1 if center_x < center_image else 1
        return 0

    def get_search_pattern_cmd(self, step):
        """Genera un comando de movimiento basado en un patrón de búsqueda"""
        twist = Twist()
        pattern_size = 10  # Número de pasos en el patrón
        
        # Patrón simple: alterna entre izquierda y derecha
        if step % (2 * pattern_size) < pattern_size:
            twist.angular.z = self.angular_speed  # Gira izquierda
        else:
            twist.angular.z = -self.angular_speed  # Gira derecha
            
        # Avanza ligeramente en algunos pasos
        if step % 3 == 0:
            twist.linear.x = 0.1
            
        return twist

    def buscar_lata_callback(self, req):
        rospy.loginfo("Servicio buscar_lata iniciado")
        response = BuscarLataResponse()
        response.lata_encontrada = False
        response.centro_x = -1.0
        
        twist = Twist()
        start_time = rospy.Time.now()
        timeout = rospy.Duration(rospy.get_param('~search_timeout', 15))
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time < timeout):
            if self.lata_encontrada:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                response.lata_encontrada = True
                response.centro_x = self.centro_x_lata
                rospy.loginfo("Lata encontrada en x=%.2f", self.centro_x_lata)
                return response
            
            # Patrón de búsqueda mejorado
            twist = self.get_search_pattern_cmd(self.search_pattern_step)
            self.cmd_vel_pub.publish(twist)
            self.search_pattern_step += 1
            
            rate.sleep()

        # Timeout alcanzado
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.logwarn("Timeout: No se encontró la lata")
        return response

if __name__ == '__main__':
    try:
        service = ImprovedBlackCanService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminado")