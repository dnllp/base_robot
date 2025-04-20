#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        # Parámetros configurables
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # Distancia mínima en metros
        self.max_speed = rospy.get_param('~max_speed', 0.3)        # Velocidad lineal máxima
        self.max_turn = rospy.get_param('~max_turn', 1.0)          # Velocidad angular máxima
        self.sector_angle = rospy.get_param('~sector_angle', 60)   # Ángulo del sector frontal (grados)
        
        # Suscriptor y publicador
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Variables de control
        self.obstacle_detected = False
        self.obstacle_angle = 0
        self.obstacle_distance = 0
        
        rospy.loginfo("Nodo de evasión de obstáculos iniciado")

    def scan_callback(self, msg):
        # Convertir ángulos a grados para facilitar el procesamiento
        angle_min = np.degrees(msg.angle_min)
        angle_max = np.degrees(msg.angle_max)
        angle_increment = np.degrees(msg.angle_increment)
        
        # Crear array de ángulos
        angles = np.arange(angle_min, angle_max, angle_increment)
        
        # Sector frontal importante para navegación
        front_sector = np.abs(angles) < (self.sector_angle/2)
        
        # Encontrar la distancia mínima en el sector frontal
        front_ranges = np.array(msg.ranges)[front_sector]
        valid_ranges = front_ranges[~np.isnan(front_ranges) & ~np.isinf(front_ranges)]
        
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            min_index = np.argmin(front_ranges)
            obstacle_angle = angles[front_sector][min_index]
            
            if min_distance < self.safe_distance:
                self.obstacle_detected = True
                self.obstacle_distance = min_distance
                self.obstacle_angle = obstacle_angle
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def avoid_obstacles(self):
        rate = rospy.Rate(10)  # 10Hz
        cmd_msg = Twist()
        
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Lógica de evasión
                if self.obstacle_angle > 0:
                    # Obstáculo a la derecha, girar a la izquierda
                    cmd_msg.linear.x = self.max_speed * 0.3
                    cmd_msg.angular.z = self.max_turn
                else:
                    # Obstáculo a la izquierda, girar a la derecha
                    cmd_msg.linear.x = self.max_speed * 0.3
                    cmd_msg.angular.z = -self.max_turn
                
                rospy.loginfo(f"Obstáculo detectado: {self.obstacle_distance:.2f}m a {self.obstacle_angle:.1f}°")
            else:
                # Movimiento hacia adelante
                cmd_msg.linear.x = self.max_speed
                cmd_msg.angular.z = 0
            
            self.cmd_pub.publish(cmd_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        oa = ObstacleAvoidance()
        oa.avoid_obstacles()
    except rospy.ROSInterruptException:
        pass