#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mi_robot.srv import EvadirObstaculo, EvadirObstaculoResponse
from enum import Enum

class EvasionState(Enum):
    IDLE = 0
    EVADING = 1
    COMPLETE = 2
    FAILED = 3

class OptimizedObstacleAvoidance:
    def __init__(self):
        rospy.init_node('optimized_obstacle_avoidance_service')
        
        # Cargar parámetros configurables
        self.load_parameters()
        
        # Variables de estado
        self.state = EvasionState.IDLE
        self.ultrasonic_ranges = {
            'front_left': float('inf'),
            'front_right': float('inf'),
            'rear_left': float('inf'),
            'rear_right': float('inf')
        }
        self.lidar_ranges = None
        self.lidar_angles = None
        self.last_obstacle_detection = rospy.Time.now()
        
        # Suscriptores
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.ultrasonic_subs = [
            rospy.Subscriber('/ultrasonic/front_left', Range, self.ultrasonic_callback, 'front_left'),
            rospy.Subscriber('/ultrasonic/front_right', Range, self.ultrasonic_callback, 'front_right'),
            rospy.Subscriber('/ultrasonic/rear_left', Range, self.ultrasonic_callback, 'rear_left'),
            rospy.Subscriber('/ultrasonic/rear_right', Range, self.ultrasonic_callback, 'rear_right')
        ]
        
        # Publicadores
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('/debug/obstacle_detection', Bool, queue_size=1)
        
        # Servicio
        self.evasion_service = rospy.Service('evadir_obstaculo', EvadirObstaculo, self.evasion_callback)
        
        rospy.loginfo("Servicio de evasión de obstáculos optimizado iniciado")

    def load_parameters(self):
        """Carga parámetros configurables desde el servidor de parámetros"""
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # 50cm
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)        # rad/s
        self.scan_sector = np.radians(rospy.get_param('~scan_sector', 60))  # convertido a radianes
        self.evasion_duration = rospy.get_param('~evasion_duration', 2.0) # segundos
        self.detection_timeout = rospy.get_param('~detection_timeout', 1.0) # segundos
        self.min_obstacle_size = rospy.get_param('~min_obstacle_size', 0.2) # metros
        self.safety_margin = rospy.get_param('~safety_margin', 0.1) # metros adicionales

    def ultrasonic_callback(self, msg, sensor_id):
        """Callback para sensores ultrasónicos con filtrado básico"""
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.ultrasonic_ranges[sensor_id] = msg.range

    def lidar_callback(self, msg):
        """Callback optimizado para LIDAR con preprocesamiento"""
        self.lidar_ranges = np.array(msg.ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        # Precalcular sector frontal
        self.front_sector_mask = (np.abs(self.lidar_angles) < (self.scan_sector/2)) & (
            ~np.isnan(self.lidar_ranges)) & (~np.isinf(self.lidar_ranges))

    def check_obstacles(self):
        """Verificación optimizada de obstáculos con múltiples sensores"""
        obstacle_detected = False
        debug_msg = Bool()
        
        # Verificar ultrasonidos frontales
        front_clear = (self.ultrasonic_ranges['front_left'] > (self.safe_distance + self.safety_margin) and 
                      self.ultrasonic_ranges['front_right'] > (self.safe_distance + self.safety_margin))
        
        if not front_clear:
            rospy.logdebug("Obstáculo detectado por ultrasonido")
            obstacle_detected = True
        
        # Verificar LIDAR si está disponible
        if self.lidar_ranges is not None and self.front_sector_mask is not None:
            front_ranges = self.lidar_ranges[self.front_sector_mask]
            if len(front_ranges) > 0:
                min_distance = np.min(front_ranges)
                if min_distance < (self.safe_distance + self.safety_margin):
                    rospy.logdebug(f"Obstáculo detectado por LIDAR a {min_distance:.2f}m")
                    obstacle_detected = True
        
        debug_msg.data = obstacle_detected
        self.debug_pub.publish(debug_msg)
        
        return obstacle_detected

    def determine_evasion_direction(self):
        """Determina la mejor dirección para evadir basado en múltiples sensores"""
        # Prioridad 1: Usar diferencia de ultrasonidos frontales
        left_dist = self.ultrasonic_ranges['front_left']
        right_dist = self.ultrasonic_ranges['front_right']
        
        if left_dist != right_dist:
            return -1 if left_dist > right_dist else 1  # -1 = izquierda, 1 = derecha
        
        # Prioridad 2: Analizar sectores LIDAR si está disponible
        if self.lidar_ranges is not None and self.front_sector_mask is not None:
            left_sector = (self.lidar_angles < 0) & self.front_sector_mask
            right_sector = (self.lidar_angles > 0) & self.front_sector_mask
            
            left_clearance = np.mean(self.lidar_ranges[left_sector]) if np.any(left_sector) else float('inf')
            right_clearance = np.mean(self.lidar_ranges[right_sector]) if np.any(right_sector) else float('inf')
            
            if left_clearance != right_clearance:
                return -1 if left_clearance > right_clearance else 1
        
        # Default: Girar a la derecha
        return 1

    def execute_evasion(self):
        """Ejecuta la maniobra de evasión completa"""
        evasion_direction = self.determine_evasion_direction()
        twist_msg = Twist()
        
        # Fase 1: Giro inicial
        twist_msg.angular.z = evasion_direction * self.turn_speed
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(self.evasion_duration * 0.7)  # 70% del tiempo girando
        
        # Fase 2: Pequeño avance
        twist_msg.angular.z = evasion_direction * self.turn_speed * 0.5  # Reducir giro
        twist_msg.linear.x = 0.1  # Avance lento
        self.cmd_vel_pub.publish(twist_msg)
        rospy.sleep(self.evasion_duration * 0.3)  # 30% del tiempo avanzando
        
        # Detener
        self.cmd_vel_pub.publish(Twist())
        return True

    def evasion_callback(self, req):
        """Servicio optimizado para evasión de obstáculos"""
        response = EvadirObstaculoResponse()
        self.state = EvasionState.IDLE
        
        if not self.check_obstacles():
            response.obstaculo_evadido = False
            response.mensaje = "No se detectaron obstáculos cercanos"
            return response
        
        self.state = EvasionState.EVADING
        rospy.loginfo("Iniciando maniobra de evasión de obstáculos")
        
        try:
            success = self.execute_evasion()
            
            if success:
                self.state = EvasionState.COMPLETE
                response.obstaculo_evadido = True
                response.mensaje = "Evación completada con éxito"
                rospy.loginfo(response.mensaje)
            else:
                self.state = EvasionState.FAILED
                response.obstaculo_evadido = False
                response.mensaje = "Fallo en la evasión"
                rospy.logwarn(response.mensaje)
            
            return response
            
        except Exception as e:
            self.state = EvasionState.FAILED
            rospy.logerr(f"Error durante la evasión: {str(e)}")
            response.obstaculo_evadido = False
            response.mensaje = f"Error: {str(e)}"
            return response
        finally:
            # Asegurar que el robot se detenga
            self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        oa_service = OptimizedObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de evasión terminado")
    except Exception as e:
        rospy.logerr(f"Error fatal en el nodo de evasión: {str(e)}")