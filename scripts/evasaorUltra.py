#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        
        # Parámetros configurables
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # 50cm
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)        # rad/s
        self.scan_sector = rospy.get_param('~scan_sector', 60)       # grados
        
        # Variables de estado
        self.obstacle_detected = False
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
        
        # Publicadores
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.evasion_pub = rospy.Publisher('/evasor', Bool, queue_size=1)
        
        rospy.loginfo("Nodo de evasión de obstáculos iniciado")

    def ultrasonic_callback(self, msg, sensor_id):
        """Callback para datos de sensores ultrasónicos"""
        self.ultrasonic_ranges[sensor_id] = msg.range
        self.check_obstacles()

    def lidar_callback(self, msg):
        """Callback para datos del LIDAR"""
        # Convertir ángulos a grados
        angle_min = np.degrees(msg.angle_min)
        angle_max = np.degrees(msg.angle_max)
        angle_inc = np.degrees(msg.angle_increment)
        
        # Crear array de ángulos
        angles = np.arange(angle_min, angle_max, angle_inc)
        
        # Sector frontal importante
        front_sector = np.abs(angles) < (self.scan_sector/2)
        
        # Obtener distancias en sector frontal
        front_ranges = np.array(msg.ranges)
        front_ranges = front_ranges[front_sector]
        front_ranges = front_ranges[~np.isnan(front_ranges) & ~np.isinf(front_ranges)]
        
        if len(front_ranges) > 0:
            min_lidar_dist = np.min(front_ranges)
            if min_lidar_dist < self.safe_distance:
                self.obstacle_detected = True
                self.evade_obstacle(min_lidar_dist)
            else:
                self.check_obstacles()

    def check_obstacles(self):
        """Verifica obstáculos en todos los sensores ultrasónicos"""
        min_distance = min(self.ultrasonic_ranges.values())
        
        if min_distance < self.safe_distance:
            self.obstacle_detected = True
            self.evade_obstacle(min_distance)
        elif self.obstacle_detected:
            self.obstacle_detected = False
            self.stop_evasion()

    def evade_obstacle(self, distance):
        """Lógica principal de evasión de obstáculos"""
        evasion_msg = Bool()
        evasion_msg.data = True
        self.evasion_pub.publish(evasion_msg)
        
        cmd_msg = Twist()
        
        # Determinar dirección de giro basado en la posición del obstáculo
        if (self.ultrasonic_ranges['front_left'] < self.ultrasonic_ranges['front_right'] or
            self.ultrasonic_ranges['rear_left'] < self.ultrasonic_ranges['rear_right']):
            # Girar a la derecha (obstáculo a la izquierda)
            cmd_msg.angular.z = -self.turn_speed
        else:
            # Girar a la izquierda (obstáculo a la derecha)
            cmd_msg.angular.z = self.turn_speed
        
        # Retroceder si hay obstáculo cerca atrás
        if (self.ultrasonic_ranges['rear_left'] < self.safe_distance or 
            self.ultrasonic_ranges['rear_right'] < self.safe_distance):
            cmd_msg.linear.x = -0.1  # Retroceder lentamente
        
        self.cmd_vel_pub.publish(cmd_msg)

    def stop_evasion(self):
        """Detiene la evasión y permite continuar"""
        evasion_msg = Bool()
        evasion_msg.data = False
        self.evasion_pub.publish(evasion_msg)
        
        cmd_msg = Twist()
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        oa = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass