#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler

class DifferentialRobot:
    def __init__(self):
        # Inicializar nodo ROS
        rospy.init_node('differential_robot_node')

        # Parametros configurables
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.065)   # Radio de rueda en metros
        self.wheel_base = rospy.get_param('~wheel_base', 0.42)     # Distancia entre ruedas en metros
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks', 341)  # Ticks por revolucion
        self.ticks_left = 0
        self.ticks_right = 0
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0
        self.left_distance = 0.0
        self.right_distance = 0.0

        # Variables de estado de odometria
        self.x = 0.0   # Posicion x en metros
        self.y = 0.0   # Posicion y en metros
        self.theta = 0.0 # Orientacion en radianes
        self.last_time = rospy.Time.now()

        # Publicador para el topico cmd_vel para Arduino
        self.cmd_vel_pub_arduino = rospy.Publisher('/cmd_vel', String, queue_size=10)

        # Suscriptores
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/encoders', Int32MultiArray, self.encoders_callback)
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)

        # Publicadores
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.imu_pub_filtered = rospy.Publisher('imu/data', Imu, queue_size=10) # Publicar IMU para otros nodos
        self.odom_broadcaster = TransformBroadcaster()

        # Temporizador para actualizar odometria
        rospy.Timer(rospy.Duration(0.02), self.update_odometry)

        rospy.loginfo("Nodo de robot diferencial inicializado.")

    def cmd_vel_callback(self, msg):
        """
        Callback para comandos de velocidad Twist (geometry_msgs/Twist)
        Convierte velocidades lineales y angulares a un formato para Arduino
        """
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calcular velocidades de rueda (diferential drive kinematics)
        left_speed = linear_vel - angular_vel * self.wheel_base / 2.0
        right_speed = linear_vel + angular_vel * self.wheel_base / 2.0

        # Enviar comando de velocidad al Arduino (formato "L<vel_int> R<vel_int>\n")
        # Puedes ajustar la escala de velocidad segun sea necesario para tu Arduino
        left_pwm = int(left_speed * 100)
        right_pwm = int(right_speed * 100)
        command = "L{} R{}\n".format(left_pwm, right_pwm)
        self.cmd_vel_pub_arduino.publish(String(data=command))

    def encoders_callback(self, msg):
        """
        Callback para los datos del encoder (std_msgs/Int32MultiArray)
        """
        if len(msg.data) == 2:
            delta_left = msg.data[0]
            delta_right = msg.data[1]

            # Actualizar el numero total de ticks
            self.ticks_left += delta_left
            self.ticks_right += delta_right

            # Calcular la distancia recorrida por cada rueda desde la ultima lectura
            distance_left = (2 * math.pi * self.wheel_radius * delta_left) / self.encoder_ticks_per_rev
            distance_right = (2 * math.pi * self.wheel_radius * delta_right) / self.encoder_ticks_per_rev

            self.left_distance += distance_left
            self.right_distance += distance_right
        else:
            rospy.logwarn("Mensaje de encoder mal formado: {}".format(msg.data))

    def imu_callback(self, msg):
        """
        Callback para los datos de la IMU (sensor_msgs/Imu)
        Simplemente re-publica los datos para otros nodos que puedan necesitarlos.
        La odometria se basa principalmente en los encoders en este ejemplo.
        Si deseas fusionar la IMU para una odometria mas precisa, deberas implementar un filtro
        (por ejemplo, filtro de Kalman o filtro de particulas).
        """
        self.imu_pub_filtered.publish(msg)

    def update_odometry(self, event):
        """
        Calcula y publica la odometria basada en los datos del encoder
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calcular el cambio en la distancia recorrida por cada rueda
        delta_left = self.left_distance
        delta_right = self.right_distance
        self.left_distance = 0.0
        self.right_distance = 0.0

        # Calcular el desplazamiento lineal y angular del robot
        linear_displacement = (delta_right + delta_left) / 2.0
        angular_displacement = (delta_right - delta_left) / self.wheel_base

        # Actualizar la pose del robot
        self.theta += angular_displacement
        self.x += linear_displacement * math.cos(self.theta)
        self.y += linear_displacement * math.sin(self.theta)

        # Crear mensaje de odometria
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                 0, 0.1, 0, 0, 0, 0,
                                 0, 0, 0.1, 0, 0, 0,
                                 0, 0, 0, 0.2, 0, 0,
                                 0, 0, 0, 0, 0.2, 0,
                                 0, 0, 0, 0, 0, 0.2]

        # Velocidad (estimada a partir del desplazamiento en dt)
        odom.twist.twist.linear.x = linear_displacement / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = angular_displacement / dt if dt > 0 else 0.0
        odom.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.02, 0, 0,
                                 0, 0, 0, 0, 0.02, 0,
                                 0, 0, 0, 0, 0, 0.02]

        # Publicar odometria
        self.odom_pub.publish(odom)

        # Enviar transformacion TF
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            q,
            current_time,
            "base_link",
            "odom"
        )

        self.last_time = current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = DifferentialRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass