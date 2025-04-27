#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import actionlib
from mi_robot.msg import BuscarLataAction, BuscarLataResult, BuscarLataFeedback

class ImprovedBlackCanActionServer:
    def __init__(self):
        rospy.init_node('improved_black_can_action_server')
        self.bridge = CvBridge()

        # Cargar parámetros configurables
        self.load_parameters()

        self.image_sub = rospy.Subscriber('/camera/front/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.action_server = actionlib.SimpleActionServer(
            'buscar_lata',
            BuscarLataAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.action_server.start()

        self.lata_encontrada = False
        self.centro_x_lata = -1.0
        self.last_detection_time = rospy.Time(0)
        self.search_pattern_step = 0
        self.current_goal = None # Para manejar la cancelación

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
        self.search_timeout = rospy.get_param('~search_timeout', 15.0) # segundos para la acción

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

    def execute_callback(self, goal):
        rospy.loginfo("Acción 'buscar_lata' iniciada")
        self.current_goal = goal
        result = BuscarLataResult()
        twist = Twist()
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz

        self.lata_encontrada = False # Resetear el estado de la lata para cada goal
        self.centro_x_lata = -1.0
        self.search_pattern_step = 0

        while not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Acción 'buscar_lata' preempted")
                self.action_server.set_preempted()
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            if self.lata_encontrada:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                result.lata_encontrada = True
                result.centro_x = self.centro_x_lata
                rospy.loginfo("Lata encontrada en x=%.2f", self.centro_x_lata)
                self.action_server.set_succeeded(result)
                return

            # Patrón de búsqueda mejorado
            twist = self.get_search_pattern_cmd(self.search_pattern_step)
            self.cmd_vel_pub.publish(twist)
            self.search_pattern_step += 1

            if (rospy.Time.now() - start_time).to_sec() > self.search_timeout:
                rospy.logwarn("Timeout: No se encontró la lata durante la acción")
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                result.lata_encontrada = False
                result.centro_x = -1.0
                self.action_server.set_aborted(result)
                return

            rate.sleep()

if __name__ == '__main__':
    try:
        server = ImprovedBlackCanActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminado")