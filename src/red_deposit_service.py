#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from mi_robot.srv import BuscarDescargar, BuscarDescargarResponse  # Importa tu servicio

class RedDepositService:
    def __init__(self):
        rospy.init_node('red_deposit_service')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rear/image_raw', Image, self.image_callback)
        self.distance_sub = rospy.Subscriber('/distancia', Float32, self.distance_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.descarga_pub = rospy.Publisher('/descarga', Bool, queue_size=1)
        self.buscar_descargar_service = rospy.Service('buscar_descargar', BuscarDescargar, self.buscar_descargar_callback)

        self.lower_red_lower = np.array([0, 100, 100])
        self.upper_red_lower = np.array([10, 255, 255])
        self.lower_red_upper = np.array([170, 100, 100])
        self.upper_red_upper = np.array([180, 255, 255])

        self.detection_threshold_area = 100 # Minimum area for the deposit
        self.image_width = 640.0
        self.angular_speed = 0.8 # Ajusta según sea necesario
        self.linear_speed_forward = 0.2 # Ajusta según sea necesario
        self.linear_speed_backward = -0.2 # Ajusta según sea necesario
        self.centering_threshold = 0.1
        self.target_distance = 0.20 # 20 cm - Ajusta según tu sensor
        self.current_distance = None
        self.deposit_centered = False
        self.approaching = False
        self.center_x = -1
        self.position = 0

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_width = float(cv_image.shape[1])
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask_lower = cv2.inRange(hsv_image, self.lower_red_lower, self.upper_red_lower)
            mask_upper = cv2.inRange(hsv_image, self.lower_red_upper, self.upper_red_upper)
            mask = cv2.bitwise_or(mask_lower, mask_upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.detection_threshold_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        self.center_x = float(M["m10"] / M["m00"])
                        self.position = self.calculate_position(self.center_x)
                        return # Solo actualiza la posición
            self.center_x = -1
            self.position = 0
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

    def buscar_descargar_callback(self, req):
        rospy.loginfo("Servicio buscar_descargar llamado.")
        response = BuscarDescargarResponse()
        response.exito = False
        response.mensaje = "Error desconocido."

        twist = Twist()
        descarga_msg = Bool()
        descarga_msg.data = False

        try:
            self.deposit_centered = False
            self.approaching = False
            start_time = rospy.Time.now()
            timeout = rospy.Duration(30) # Tiempo máximo para buscar y acercarse

            while not rospy.is_shutdown() and (rospy.Time.now() - start_time < timeout):
                if not self.deposit_centered:
                    if self.position == 1:
                        twist.angular.z = -self.angular_speed
                        twist.linear.x = 0.0
                    elif self.position == -1:
                        twist.angular.z = self.angular_speed
                        twist.linear.x = 0.0
                    else:
                        twist.angular.z = 0.0
                        twist.linear.x = 0.0
                        self.deposit_centered = True
                        rospy.loginfo("Deposito rojo centrado.")
                        self.approaching = True
                elif self.approaching:
                    if self.current_distance is not None:
                        if self.current_distance > self.target_distance + 0.05: # Margen de seguridad
                            twist.linear.x = self.linear_speed_backward
                            twist.angular.z = 0.0
                        elif self.current_distance < self.target_distance - 0.05:
                            twist.linear.x = self.linear_speed_forward
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            descarga_msg.data = True
                            self.descarga_pub.publish(descarga_msg)
                            rospy.loginfo("Deposito ubicado. Iniciando descarga.")
                            rospy.sleep(5)  # Simula la descarga
                            response.exito = True
                            response.mensaje = "Descarga completada con exito."
                            self.deposit_centered = False
                            self.approaching = False
                            return response # Sale del servicio
                    else:
                        rospy.logwarn_once("Esperando la distancia al deposito /distancia")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)

            response.mensaje = "Tiempo de espera agotado durante la busqueda/acercamiento."
            return response

        except Exception as e:
            rospy.logerr("Error durante la busqueda/descarga: %s", str(e))
            response.mensaje = str(e)
            return response

if __name__ == '__main__':
    try:
        service = RedDepositService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass