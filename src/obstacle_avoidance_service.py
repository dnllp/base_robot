#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mi_robot.srv import EvadirObstaculo, EvadirObstaculoResponse

class ObstacleAvoidanceService:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_service')

        # Parámetros configurables
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # 50cm
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)        # rad/s
        self.scan_sector = rospy.get_param('~scan_sector', 60)      # grados
        self.evasion_duration = rospy.get_param('~evasion_duration', 2.0) # segundos de evasión

        # Variables de estado
        self.ultrasonic_ranges = {
            'front_left': float('inf'),
            'front_right': float('inf'),
            'rear_left': float('inf'),
            'rear_right': float('inf')
        }

        # Suscriptores
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/ultrasonic/front_left', Range, self.ultrasonic_callback, 'front_left')
        rospy.Subscriber('/ultrasonic/front_right', Range, self.ultrasonic_callback, 'front_right')
        rospy.Subscriber('/ultrasonic/rear_left', Range, self.ultrasonic_callback, 'rear_left')
        rospy.Subscriber('/ultrasonic/rear_right', Range, self.ultrasonic_callback, 'rear_right')

        # Publicadores (solo cmd_vel para controlar el movimiento durante la evasión)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.evadir_obstaculo_service = rospy.Service('evadir_obstaculo', EvadirObstaculo, self.evadir_obstaculo_callback)

        rospy.loginfo("Nodo de servicio de evasión de obstáculos iniciado")
        self.last_lidar_ranges = None

    def ultrasonic_callback(self, msg, sensor_id):
        """Callback para datos de sensores ultrasónicos"""
        self.ultrasonic_ranges[sensor_id] = msg.range

    def lidar_callback(self, msg):
        """Callback para datos del LIDAR"""
        self.last_lidar_ranges = msg.ranges

    def check_immediate_obstacle(self):
        """Verifica si hay un obstáculo inmediato usando LIDAR y ultrasonido"""
        # Chequear LIDAR
        if self.last_lidar_ranges is not None:
            angle_min = np.degrees(msg.angle_min)
            angle_max = np.degrees(msg.angle_max)
            angle_inc = np.degrees(msg.angle_increment)
            angles = np.arange(angle_min, angle_max, angle_inc)
            front_sector = np.abs(angles) < (self.scan_sector/2)
            front_ranges = np.array(self.last_lidar_ranges)
            front_ranges = front_ranges[front_sector]
            front_ranges = front_ranges[~np.isnan(front_ranges) & ~np.isinf(front_ranges)]
            if len(front_ranges) > 0 and np.min(front_ranges) < self.safe_distance:
                return True

        # Chequear ultrasonido frontal
        if self.ultrasonic_ranges['front_left'] < self.safe_distance or self.ultrasonic_ranges['front_right'] < self.safe_distance:
            return True

        return False

    def evade_once(self):
        """Realiza una pequeña maniobra de evasión"""
        cmd_msg = Twist()

        # Determinar dirección de giro (simple lógica basada en ultrasonido frontal)
        if self.ultrasonic_ranges['front_left'] < self.ultrasonic_ranges['front_right']:
            # Girar a la derecha
            cmd_msg.angular.z = -self.turn_speed
        else:
            # Girar a la izquierda
            cmd_msg.angular.z = self.turn_speed

        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(self.evasion_duration)

        # Detener el movimiento
        self.cmd_vel_pub.publish(Twist())
        return True

    def evadir_obstaculo_callback(self, req):
        """Callback para el servicio de evasión de obstáculos"""
        rospy.loginfo("Servicio evadir_obstaculo llamado.")
        response = EvadirObstaculoResponse()
        response.obstaculo_evadido = False
        response.mensaje = ""

        if self.check_immediate_obstacle():
            rospy.loginfo("Obstáculo detectado. Iniciando maniobra de evasión.")
            if self.evade_once():
                response.obstaculo_evadido = True
                response.mensaje = "Maniobra de evasión completada."
            else:
                response.mensaje = "Fallo al completar la maniobra de evasión."
        else:
            response.mensaje = "No se detectó ningún obstáculo cercano."

        return response

if __name__ == '__main__':
    try:
        oa_service = ObstacleAvoidanceService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass