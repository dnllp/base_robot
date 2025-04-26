#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import tf

class DifferentialRobotFusion:
    def __init__(self):
        rospy.init_node('differential_robot_fusion_node')

        # Parámetros del robot
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.065)
        self.wheel_base = rospy.get_param('~wheel_base', 0.42)
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks', 341)
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        # Publicador para comandos de velocidad al Arduino
        self.cmd_vel_pub_arduino = rospy.Publisher('/cmd_vel', String, queue_size=10)

        # Publicador para la odometría basada en encoders (para robot_localization)
        self.encoder_odom_pub = rospy.Publisher('encoder_odom', Odometry, queue_size=10)

        # Suscriptores
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/encoders', Int32MultiArray, self.encoders_callback)
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)

        # Broadcaster para la transformacion tf de la odometria del encoder
        self.encoder_odom_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("Nodo de fusion de odometria inicializado.")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        left_speed = linear_vel - angular_vel * self.wheel_base / 2.0
        right_speed = linear_vel + angular_vel * self.wheel_base / 2.0
        left_pwm = int(left_speed * 100)
        right_pwm = int(right_speed * 100)
        command = "L{} R{}\n".format(left_pwm, right_pwm)
        self.cmd_vel_pub_arduino.publish(String(data=command))

    def encoders_callback(self, msg):
        if len(msg.data) == 2:
            current_ticks_left = msg.data[0]
            current_ticks_right = msg.data[1]

            delta_ticks_left = current_ticks_left - self.prev_ticks_left
            delta_ticks_right = current_ticks_right - self.prev_ticks_right

            self.prev_ticks_left = current_ticks_left
            self.prev_ticks_right = current_ticks_right

            distance_left = (2 * math.pi * self.wheel_radius * delta_ticks_left) / self.encoder_ticks_per_rev
            distance_right = (2 * math.pi * self.wheel_radius * delta_ticks_right) / self.encoder_ticks_per_rev

            # Estimar la pose basada en encoders (similar a tu update_odometry anterior)
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt == 0:
                return

            linear_displacement = (distance_right + distance_left) / 2.0
            angular_displacement = (distance_right - distance_left) / self.wheel_base

            self.theta += angular_displacement
            self.x += linear_displacement * math.cos(self.theta)
            self.y += linear_displacement * math.sin(self.theta)
            self.last_time = current_time

            # Publicar la odometría del encoder
            encoder_odom = Odometry()
            encoder_odom.header.stamp = current_time
            encoder_odom.header.frame_id = "odom_encoder"  # Un frame ID diferente
            encoder_odom.child_frame_id = "base_link"

            encoder_odom.pose.pose.position.x = self.x
            encoder_odom.pose.pose.position.y = self.y
            encoder_odom.pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, self.theta)
            encoder_odom.pose.pose.orientation.x = q[0]
            encoder_odom.pose.pose.orientation.y = q[1]
            encoder_odom.pose.pose.orientation.z = q[2]
            encoder_odom.pose.pose.orientation.w = q[3]
            # Ajusta las covarianzas según la confianza en tus encoders
            encoder_odom.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                             0, 0.01, 0, 0, 0, 0,
                                             0, 0, 0.01, 0, 0, 0,
                                             0, 0, 0, 0.01, 0, 0,
                                             0, 0, 0, 0, 0.01, 0,
                                             0, 0, 0, 0, 0, 0.01]

            # Publicar la velocidad (opcional, si puedes estimarla a partir de los ticks/tiempo)
            encoder_odom.twist.twist.linear.x = linear_displacement / dt
            encoder_odom.twist.twist.angular.z = angular_displacement / dt
            encoder_odom.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                            0, 0.01, 0, 0, 0, 0,
                                            0, 0, 0.01, 0, 0, 0,
                                            0, 0, 0, 0.01, 0, 0,
                                            0, 0, 0, 0, 0.01, 0,
                                            0, 0, 0, 0, 0, 0.01]

            self.encoder_odom_pub.publish(encoder_odom)

            # Broadcast la transformacion tf para la odometria del encoder
            self.encoder_odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                q,
                current_time,
                "base_link",
                "odom_encoder"
            )
        else:
            rospy.logwarn("Mensaje de encoder mal formado: {}".format(msg.data))

    def imu_callback(self, msg):
        # Simplemente publicamos el mensaje IMU tal cual para que robot_localization lo consuma
        self.imu_pub.publish(msg) # Asegúrate de tener self.imu_pub definido en __init__

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = DifferentialRobotFusion()
        robot.run()
    except rospy.ROSInterruptException:
        pass