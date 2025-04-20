#!/usr/bin/env python
import rospy
import serial
import math
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster

class DifferentialRobot:
    def __init__(self):
        # Inicializar nodo ROS
        rospy.init_node('differential_robot_node')
        
        # Parametros configurables
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 57600)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.065)  # Radio de rueda en metros
        self.wheel_base = rospy.get_param('~wheel_base', 0.42)  # Distancia entre ruedas en metros
        self.encoder_ticks = rospy.get_param('~encoder_ticks', 341)  # Ticks por revolucion
        self.left_distance = 0
        self.right_distance = 0
        # Variables de estado
        self.x = 0.0  # Posicion x en metros
        self.y = 0.0  # Posicion y en metros
        self.theta = 0.0  # Orientacion en radianes
        self.last_time = rospy.Time.now()
        
        # Inicializar comunicacion serial con Arduino
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo("Conectado a Arduino en %s", self.port)
        except serial.SerialException as e:
            rospy.logerr("Error al conectar con Arduino: %s", str(e))
            rospy.signal_shutdown("Error de conexion serial")
            return
        
        # Suscriptores y Publicadores
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.odom_broadcaster = TransformBroadcaster()
        
        # Temporizador para actualizar odometria
        rospy.Timer(rospy.Duration(0.02), self.update_odometry)
        
    def cmd_vel_callback(self, msg):
        """
        Callback para comandos de velocidad Twist (geometry_msgs/Twist)
        Convierte velocidades lineales y angulares a velocidades de rueda
        """
        # Calcular velocidades de rueda (diferential drive kinematics)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        left_speed = linear_vel - angular_vel * self.wheel_base / 2.0
        right_speed = linear_vel + angular_vel * self.wheel_base / 2.0
        
        # Convertir a RPM (opcional, depende de tu controlador)
        left_rpm = (left_speed * 60) / (2 * math.pi * self.wheel_radius)
        right_rpm = (right_speed * 60) / (2 * math.pi * self.wheel_radius)
        
        # Enviar comandos a Arduino (formato: "L100 R100\n")
        command = "L{} R{}\n".format(int(left_rpm), int(right_rpm))
        try:
            self.serial_conn.write(command.encode())
        except serial.SerialException as e:
            rospy.logwarn("Error al enviar comando a Arduino: %s", str(e))
    
    def parse_serial_data(self, line):
        """
        Procesa datos recibidos del Arduino (encoders e IMU)
        Formato esperado: "E<left_ticks>,<right_ticks> I<qx>,<qy>,<qz>,<qw>"
        """
        if line.startswith('E'):
            # Datos de encoder
            encoder_data = line[1:].split(' I')[0]
            left_ticks, right_ticks = map(int, encoder_data.split(','))
            self.update_encoder_data(left_ticks, right_ticks)
            
        if 'I' in line:
            # Datos de IMU (cuaternion)
            imu_data = line.split('I')[1]
            qx, qy, qz, qw = map(float, imu_data.split(','))
            self.publish_imu_data(qx, qy, qz, qw)
    
    def update_encoder_data(self, left_ticks, right_ticks):
        """
        Actualiza la odometria basada en los ticks del encoder
        """
        # Calcular distancia recorrida por cada rueda
        left_distance = (2 * math.pi * self.wheel_radius * left_ticks) / self.encoder_ticks
        right_distance = (2 * math.pi * self.wheel_radius * right_ticks) / self.encoder_ticks
        
        # Almacenar para calculo de odometria en el timer
        self.left_distance = left_distance
        self.right_distance = right_distance
    
    def publish_imu_data(self, qx, qy, qz, qw):
        """
        Publica datos de la IMU como mensaje ROS Imu
        """
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        # Orientacion (cuaternion)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        
        # Covarianza (valores tipicos para BNO055)
        imu_msg.orientation_covariance = [0.01, 0, 0,
                                          0, 0.01, 0,
                                          0, 0, 0.01]
        
        self.imu_pub.publish(imu_msg)
    
    def update_odometry(self, event):
        """
        Calcula y publica la odometria fusionando encoders e IMU
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Calcular desplazamiento lineal y angular (modelo diferencial)
        linear = (self.right_distance + self.left_distance) / 2.0
        angular = (self.right_distance - self.left_distance) / self.wheel_base
        
        # Actualizar posicion (integracion simple)
        self.theta += angular
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        
        # Crear mensaje de odometria
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Posicion
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientacion (convertir angulo a cuaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Covarianza (ajustar segun confianza en los sensores)
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.2, 0, 0,
                               0, 0, 0, 0, 0.2, 0,
                               0, 0, 0, 0, 0, 0.2]
        
        # Velocidad (opcional, si los encoders reportan velocidad)
        odom.twist.twist.linear.x = linear / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = angular / dt if dt > 0 else 0.0
        
        # Publicar odometria
        self.odom_pub.publish(odom)
        
        # Enviar transformada TF
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            (0, 0, math.sin(self.theta/2), math.cos(self.theta/2)),
            current_time,
            "base_link",
            "odom"
        )
        
        self.last_time = current_time
    
    def run(self):
        """
        Bucle principal que lee datos seriales del Arduino
        """
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            if self.serial_conn.in_waiting:
                try:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    self.parse_serial_data(line)
                except UnicodeDecodeError as e:
                    rospy.logwarn("Error al decodificar datos seriales: %s", str(e))
            rate.sleep()

if __name__ == '__main__':
    try:
        robot = DifferentialRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
