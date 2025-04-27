#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from mi_robot.srv import BuscarDescargar, BuscarDescargarResponse
from enum import Enum

class State(Enum):
    SEARCHING = 0
    CENTERING = 1
    APPROACHING = 2
    DUMPING = 3
    COMPLETE = 4
    FAILED = 5

class RedDepositService:
    def __init__(self):
        rospy.init_node('red_deposit_service')
        self.bridge = CvBridge()
        
        # Cargar parámetros configurables
        self.load_parameters()
        
        # Subs y pubs
        self.image_sub = rospy.Subscriber('/camera/rear/image_raw', Image, self.image_callback)
        self.distance_sub = rospy.Subscriber('/distancia', Float32, self.distance_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.descarga_pub = rospy.Publisher('/descarga', Bool, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/debug/deposit_detection', Image, queue_size=1)
        
        # Servicios
        self.buscar_descargar_service = rospy.Service('buscar_descargar', BuscarDescargar, self.buscar_descargar_callback)
        
        # Variables de estado
        self.state = State.SEARCHING
        self.current_distance = None
        self.center_x = -1
        self.position = 0
        self.last_detection_time = rospy.Time.now()
        
        # Para visualización
        cv2.namedWindow('Deposit Detection')
        cv2.createTrackbar('H Min Lower', 'Deposit Detection', self.lower_red_lower[0], 180, self.nothing)
        cv2.createTrackbar('S Min Lower', 'Deposit Detection', self.lower_red_lower[1], 255, self.nothing)
        cv2.createTrackbar('V Min Lower', 'Deposit Detection', self.lower_red_lower[2], 255, self.nothing)
        cv2.createTrackbar('H Max Lower', 'Deposit Detection', self.upper_red_lower[0], 180, self.nothing)
        cv2.createTrackbar('S Max Lower', 'Deposit Detection', self.upper_red_lower[1], 255, self.nothing)
        cv2.createTrackbar('V Max Lower', 'Deposit Detection', self.upper_red_lower[2], 255, self.nothing)
        
        rospy.loginfo("Nodo de depósito rojo inicializado")

    def nothing(self, x):
        pass

    def load_parameters(self):
        """Carga parámetros configurables desde el servidor de parámetros"""
        # Rangos HSV para rojo (dos segmentos)
        self.lower_red_lower = np.array([
            rospy.get_param('~h_min_lower', 0),
            rospy.get_param('~s_min_lower', 100),
            rospy.get_param('~v_min_lower', 100)
        ])
        self.upper_red_lower = np.array([
            rospy.get_param('~h_max_lower', 10),
            rospy.get_param('~s_max_lower', 255),
            rospy.get_param('~v_max_lower', 255)
        ])
        self.lower_red_upper = np.array([
            rospy.get_param('~h_min_upper', 170),
            rospy.get_param('~s_min_upper', 100),
            rospy.get_param('~v_min_upper', 100)
        ])
        self.upper_red_upper = np.array([
            rospy.get_param('~h_max_upper', 180),
            rospy.get_param('~s_max_upper', 255),
            rospy.get_param('~v_max_upper', 255)
        ])
        
        # Parámetros de control
        self.detection_threshold_area = rospy.get_param('~detection_threshold_area', 100)
        self.angular_speed = rospy.get_param('~angular_speed', 0.8)
        self.linear_speed_forward = rospy.get_param('~linear_speed_forward', 0.2)
        self.linear_speed_backward = rospy.get_param('~linear_speed_backward', -0.2)
        self.centering_threshold = rospy.get_param('~centering_threshold', 0.1)
        self.target_distance = rospy.get_param('~target_distance', 0.20)
        self.search_timeout = rospy.get_param('~search_timeout', 30)
        self.dump_duration = rospy.get_param('~dump_duration', 5)
        self.detection_timeout = rospy.get_param('~detection_timeout', 1.0)

    def distance_callback(self, msg):
        """Callback para datos del sensor de distancia"""
        if msg.data > 0:  # Filtra valores negativos o inválidos
            self.current_distance = msg.data
        else:
            rospy.logwarn_throttle(1, "Distancia inválida recibida: %f", msg.data)

    def image_callback(self, msg):
        """Procesamiento de imagen para detectar depósito rojo"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Actualizar valores HSV desde trackbars si la ventana está abierta
            if cv2.getWindowProperty('Deposit Detection', 0) >= 0:
                self.lower_red_lower = np.array([
                    cv2.getTrackbarPos('H Min Lower', 'Deposit Detection'),
                    cv2.getTrackbarPos('S Min Lower', 'Deposit Detection'),
                    cv2.getTrackbarPos('V Min Lower', 'Deposit Detection')
                ])
                self.upper_red_lower = np.array([
                    cv2.getTrackbarPos('H Max Lower', 'Deposit Detection'),
                    cv2.getTrackbarPos('S Max Lower', 'Deposit Detection'),
                    cv2.getTrackbarPos('V Max Lower', 'Deposit Detection')
                ])
            
            # Detección de color rojo (dos segmentos HSV)
            mask_lower = cv2.inRange(hsv, self.lower_red_lower, self.upper_red_lower)
            mask_upper = cv2.inRange(hsv, self.lower_red_upper, self.upper_red_upper)
            mask = cv2.bitwise_or(mask_lower, mask_upper)
            
            # Filtrado morfológico
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # Encontrar contornos
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Procesar detección
            self.center_x = -1
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > self.detection_threshold_area:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        self.center_x = int(M["m10"] / M["m00"])
                        self.position = self.calculate_position(self.center_x, cv_image.shape[1])
                        self.last_detection_time = rospy.Time.now()
            
            # Publicar imagen para depuración
            debug_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            if self.center_x != -1:
                cv2.circle(debug_img, (self.center_x, int(cv_image.shape[0]/2)), 10, (0,255,0), 2)
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            
            # Mostrar ventana de depuración
            if cv2.getWindowProperty('Deposit Detection', 0) >= 0:
                cv2.imshow('Deposit Detection', debug_img)
                cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr("Error en image_callback: %s", str(e))

    def calculate_position(self, center_x, image_width):
        """Determina si el depósito está a la izquierda, derecha o centrado"""
        center_image = image_width / 2.0
        deviation = abs(center_x - center_image)
        tolerance = image_width * self.centering_threshold

        if deviation > tolerance:
            return -1 if center_x < center_image else 1
        return 0

    def buscar_descargar_callback(self, req):
        """Servicio principal para buscar y descargar en el depósito"""
        rospy.loginfo("Iniciando búsqueda y descarga en depósito rojo")
        response = BuscarDescargarResponse()
        self.state = State.SEARCHING
        
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown() and self.state != State.COMPLETE:
                if (rospy.Time.now() - start_time).to_sec() > self.search_timeout:
                    self.state = State.FAILED
                    response.mensaje = "Tiempo de búsqueda agotado"
                    break
                
                twist = Twist()
                
                if self.state == State.SEARCHING:
                    if self.center_x == -1:
                        # Patrón de búsqueda (alterna dirección)
                        direction = 1 if (rospy.Time.now().to_sec() % 4) < 2 else -1
                        twist.angular.z = direction * self.angular_speed
                    else:
                        self.state = State.CENTERING
                        rospy.loginfo("Depósito detectado, iniciando centrado")
                
                elif self.state == State.CENTERING:
                    if self.center_x == -1:
                        if (rospy.Time.now() - self.last_detection_time).to_sec() > self.detection_timeout:
                            self.state = State.SEARCHING
                            rospy.logwarn("Perdí de vista el depósito, volviendo a buscar")
                    elif self.position == 0:
                        self.state = State.APPROACHING
                        rospy.loginfo("Depósito centrado, iniciando acercamiento")
                    else:
                        twist.angular.z = -self.angular_speed if self.position == 1 else self.angular_speed
                
                elif self.state == State.APPROACHING:
                    if self.center_x == -1:
                        if (rospy.Time.now() - self.last_detection_time).to_sec() > self.detection_timeout:
                            self.state = State.SEARCHING
                            rospy.logwarn("Perdí de vista el depósito, volviendo a buscar")
                    elif self.current_distance is None:
                        rospy.logwarn_throttle(1, "Esperando datos de distancia")
                    else:
                        error = self.current_distance - self.target_distance
                        if abs(error) < 0.05:  # Margen de 5cm
                            self.state = State.DUMPING
                            rospy.loginfo("Distancia alcanzada, iniciando descarga")
                        else:
                            twist.linear.x = self.linear_speed_forward if error > 0 else self.linear_speed_backward
                
                elif self.state == State.DUMPING:
                    # Ejecutar descarga
                    self.descarga_pub.publish(Bool(True))
                    rospy.sleep(self.dump_duration)
                    self.descarga_pub.publish(Bool(False))
                    response.exito = True
                    response.mensaje = "Descarga completada con éxito"
                    self.state = State.COMPLETE
                
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            
            if self.state == State.FAILED:
                response.exito = False
                # Detener el robot si falló
                self.cmd_vel_pub.publish(Twist())
            
            return response
            
        except Exception as e:
            rospy.logerr("Error en buscar_descargar_callback: %s", str(e))
            response.exito = False
            response.mensaje = f"Error: {str(e)}"
            return response
        finally:
            # Asegurarse que el robot se detenga
            self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        service = RedDepositService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo terminado")
    finally:
        cv2.destroyAllWindows()