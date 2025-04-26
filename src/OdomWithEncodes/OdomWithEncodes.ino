#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NewPing.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ArduinoHardware.h>

// --- Definiciones para el Control de Motores ---
#define MOTOR_LEFT_PWM1 4
#define MOTOR_LEFT_PWM2 5
#define MOTOR_RIGHT_PWM1 6
#define MOTOR_RIGHT_PWM2 7

// --- Definiciones para Encoders ---
#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 3
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 19

// --- Definiciones para Sensores Ultrasónicos ---
#define FRONT_LEFT_TRIG 29
#define FRONT_LEFT_ECHO 28
#define FRONT_RIGHT_TRIG 27
#define FRONT_RIGHT_ECHO 26
#define REAR_LEFT_TRIG 25
#define REAR_LEFT_ECHO 24
#define REAR_RIGHT_TRIG 23
#define REAR_RIGHT_ECHO 22
#define REAR_LOW_TRIG 31
#define REAR_LOW_ECHO 30

// --- Definiciones para la Barredora (VNH2SP30) ---
const int inAPin = 10;
const int inBPin = 9;
const int pwMPin = 8;

// --- Variables para Encoders ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;
float wheel_radius = 0.065;       // Radio de las ruedas en metros 
float wheel_base = 0.42;         // Distancia entre las ruedas en metros 
int ticks_per_revolution = 341; // Ticks por revolución del encoder 
float left_distance = 0.0;
float right_distance = 0.0;

// --- IMU ---
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// --- ROS NodeHandle ---
ros::NodeHandle nh;

// --- Publicadores ---
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);

std_msgs::Int32MultiArray encoder_msg;
ros::Publisher encoder_pub("/encoders", &encoder_msg);

sensor_msgs::Range front_left_range;
ros::Publisher pub_front_left("/ultrasonic/front_left", &front_left_range);
sensor_msgs::Range front_right_range;
ros::Publisher pub_front_right("/ultrasonic/front_right", &front_right_range);
sensor_msgs::Range rear_left_range;
ros::Publisher pub_rear_left("/ultrasonic/rear_left", &rear_left_range);
sensor_msgs::Range rear_right_range;
ros::Publisher pub_rear_right("/ultrasonic/rear_right", &rear_right_range);
sensor_msgs::Range rear_low_range;
ros::Publisher pub_rear_low("/distancia", &rear_low_range);

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("/odom", &odom_msg);
tf::TransformBroadcaster odom_broadcaster;

// --- Suscriptores ---
void velocity_cb(const geometry_msgs::Twist& twist_msg);
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velocity_cb);

void barredoraCallback(const std_msgs::Bool& msg);
ros::Subscriber<std_msgs::Bool> sub_barredora("/barredora", &barredoraCallback);

// --- Variables de control de velocidad ---
float linear_x = 0;
float angular_z = 0;

// --- Variables para temporización no bloqueante de los sensores ultrasónicos ---
unsigned long previousMillisUltrasonic = 0;
const long intervalUltrasonic = 50;

// --- Objetos NewPing para sensores ultrasónicos ---
NewPing sonar_front_left(FRONT_LEFT_TRIG, FRONT_LEFT_ECHO, 200);
NewPing sonar_front_right(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO, 200);
NewPing sonar_rear_left(REAR_LEFT_TRIG, REAR_LEFT_ECHO, 200);
NewPing sonar_rear_right(REAR_RIGHT_TRIG, REAR_RIGHT_ECHO, 200);
NewPing sonar_rear_low(REAR_LOW_TRIG, REAR_LOW_ECHO, 200);

// --- Variables de odometría ---
float x = 0.0;
float y = 0.0;
float theta = 0.0;
unsigned long last_odom_update = 0;
const long odom_update_interval = 50; // Update odometry at 20Hz

// --- Funciones de configuración para mensajes Range ---
void setupRangeMessage(sensor_msgs::Range &range_msg, const char *frame_id);
void readAndPublishUltrasonic(NewPing &sonar, sensor_msgs::Range &range_msg, ros::Publisher &publisher);

// --- Funciones para la barredora ---
void activaBarredora();
void desactivaBarredora();

unsigned long last_publish_time = 0;
const unsigned long publish_interval = 20;

void setup() {
Serial.begin(57600);
nh.initNode();

// --- Publicidad de tópicos ---
nh.advertise(imu_pub);
nh.advertise(encoder_pub);
nh.advertise(pub_front_left);
nh.advertise(pub_front_right);
nh.advertise(pub_rear_left);
nh.advertise(pub_rear_right);
nh.advertise(pub_rear_low);
nh.advertise(odom_pub);

// --- Suscripción a tópicos ---
nh.subscribe(vel_sub);
nh.subscribe(sub_barredora);

// --- Configuración de pines de motor ---
pinMode(MOTOR_LEFT_PWM1, OUTPUT);
pinMode(MOTOR_LEFT_PWM2, OUTPUT);
pinMode(MOTOR_RIGHT_PWM1, OUTPUT);
pinMode(MOTOR_RIGHT_PWM2, OUTPUT);

// --- Inicialización de encoders ---
pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoder, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoder, CHANGE);

// --- Inicialización de IMU ---
if(!bno.begin()) {
Serial.println("Error al iniciar BNO055");
while(1);
}
delay(1000);
bno.setExtCrystalUse(true);

// --- Inicialización del array del encoder message ---
encoder_msg.data = (int32_t*)malloc(sizeof(int32_t) * 2);
encoder_msg.data_length = 2;

// --- Configuración de mensajes Range ---
setupRangeMessage(front_left_range, "front_left");
setupRangeMessage(front_right_range, "front_right");
setupRangeMessage(rear_left_range, "rear_left");
setupRangeMessage(rear_right_range, "rear_right");
setupRangeMessage(rear_low_range, "rear_low");

// --- Configuración de pines de la barredora ---
pinMode(inAPin, OUTPUT);
pinMode(inBPin, OUTPUT);
pinMode(pwMPin, OUTPUT);
desactivaBarredora();

Serial.println("Arduino Mega - Nodo ROS Integrado con Odometría");
}

void loop() {
unsigned long currentMillis = millis();

// --- Publicar datos de sensores periódicamente ---
if(currentMillis - last_publish_time >= publish_interval) {
publishSensorData();
last_publish_time = currentMillis;
}

// --- Lectura y publicación de sensores ultrasónicos ---
if (currentMillis - previousMillisUltrasonic >= intervalUltrasonic) {
previousMillisUltrasonic = currentMillis;
readAndPublishUltrasonic(sonar_front_left, front_left_range, pub_front_left);
readAndPublishUltrasonic(sonar_front_right, front_right_range, pub_front_right);
readAndPublishUltrasonic(sonar_rear_left, rear_left_range, pub_rear_left);
readAndPublishUltrasonic(sonar_rear_right, rear_right_range, pub_rear_right);
readAndPublishUltrasonic(sonar_rear_low, rear_low_range, pub_rear_low);
}

// --- Calcular y publicar odometría ---
if (currentMillis - last_odom_update >= odom_update_interval) {
updateOdometry();
last_odom_update = currentMillis;
}

nh.spinOnce();
delayMicroseconds(100);
}

void publishSensorData() {
// --- Leer IMU ---
imu::Quaternion quat = bno.getQuat();
imu::Vector<3> linear_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> angular_velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

// --- Llenar el mensaje IMU ---
imu_msg.header.stamp = nh.now();
imu_msg.header.frame_id = "imu_frame";
imu_msg.orientation.x = quat.x();
imu_msg.orientation.y = quat.y();
imu_msg.orientation.z = quat.z();
imu_msg.orientation.w = quat.w();
for (int i = 0; i < 9; i++) {
imu_msg.orientation_covariance[i] = 0.0;
imu_msg.angular_velocity_covariance[i] = 0.0;
imu_msg.linear_acceleration_covariance[i] = 0.0;
}
imu_msg.angular_velocity.x = angular_velocity.x();
imu_msg.angular_velocity.y = angular_velocity.y();
imu_msg.angular_velocity.z = angular_velocity.z();
imu_msg.linear_acceleration.x = linear_acceleration.x();
imu_msg.linear_acceleration.y = linear_acceleration.y();
imu_msg.linear_acceleration.z = linear_acceleration.z();
imu_pub.publish(&imu_msg);

// --- Calcular y publicar datos del encoder ---
long delta_left = left_ticks - prev_left_ticks;
long delta_right = right_ticks - prev_right_ticks;
prev_left_ticks = left_ticks;
prev_right_ticks = right_ticks;
encoder_msg.data[0] = delta_left;
encoder_msg.data[1] = delta_right;
encoder_pub.publish(&encoder_msg);
}

void velocity_cb(const geometry_msgs::Twist& twist_msg) {
linear_x = twist_msg.linear.x;
angular_z = twist_msg.angular.z;
float left_vel = (linear_x - (angular_z * wheel_base / 2.0)) / wheel_radius;
float right_vel = (linear_x + (angular_z * wheel_base / 2.0)) / wheel_radius;
int left_pwm = map(left_vel * 100, -255, 255, -255, 255);
int right_pwm = map(right_vel * 100, -255, 255, -255, 255);
setMotorSpeed(MOTOR_LEFT_PWM1, MOTOR_LEFT_PWM2, left_pwm);
setMotorSpeed(MOTOR_RIGHT_PWM1, MOTOR_RIGHT_PWM2, right_pwm);
}

void setMotorSpeed(int pwm_pin1, int pwm_pin2, int speed) {
speed = constrain(speed, -255, 255);
if(speed > 0) {
analogWrite(pwm_pin1, speed);
analogWrite(pwm_pin2, 0);
} else if(speed < 0) {
analogWrite(pwm_pin1, 0);
analogWrite(pwm_pin2, -speed);
} else {
analogWrite(pwm_pin1, 0);
analogWrite(pwm_pin2, 0);
}
}

void handleLeftEncoder() {
left_ticks += (digitalRead(ENCODER_LEFT_B)) ? (digitalRead(ENCODER_LEFT_A) ? -1 : 1) : (digitalRead(ENCODER_LEFT_A) ? 1 : -1);
}

void handleRightEncoder() {
right_ticks += (digitalRead(ENCODER_RIGHT_B)) ? (digitalRead(ENCODER_RIGHT_A) ? 1 : -1) : (digitalRead(ENCODER_RIGHT_A) ? -1 : 1);
}

void setupRangeMessage(sensor_msgs::Range &range_msg, const char *frame_id) {
range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg.header.frame_id = frame_id;
range_msg.field_of_view = 0.26;
range_msg.min_range = 0.02;
range_msg.max_range = 2.0;
}

void readAndPublishUltrasonic(NewPing &sonar, sensor_msgs::Range &range_msg, ros::Publisher &publisher) {
unsigned int cm = sonar.ping_cm();
range_msg.range = cm / 100.0;
range_msg.header.stamp = nh.now();
publisher.publish(&range_msg);
}

void activaBarredora() {
digitalWrite(inAPin, LOW);
digitalWrite(inBPin, HIGH);
analogWrite(pwMPin, 255);
}

void desactivaBarredora() {
digitalWrite(inAPin, LOW);
digitalWrite(inBPin, LOW);
analogWrite(pwMPin, 0);
}

void barredoraCallback(const std_msgs::Bool& msg) {
if (msg.data) {
activaBarredora();
Serial.println("Barredora activada.");
} else {
desactivaBarredora();
Serial.println("Barredora desactivada.");
}
}

void updateOdometry() {
// Calcular la distancia recorrida por cada rueda
long current_left_ticks = left_ticks;
  long current_right_ticks = right_ticks;

  long delta_left = current_left_ticks - prev_left_ticks;
  long delta_right = current_right_ticks - prev_right_ticks;

  prev_left_ticks = current_left_ticks;
  prev_right_ticks = current_right_ticks;

  float d_left = 2 * M_PI * wheel_radius * delta_left / ticks_per_revolution;
  float d_right = 2 * M_PI * wheel_radius * delta_right / ticks_per_revolution;

  float d_theta = (d_right - d_left) / wheel_base;
  float d_x = (d_left + d_right) / 2.0 * cos(theta);
  float d_y = (d_left + d_right) / 2.0 * sin(theta);

  x += d_x;
  y += d_y;
  theta += d_theta;

  // Publicar mensaje de odometría
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

  // Covarianza de la pose (ajusta según la confianza en la odometría)
  odom_msg.pose.covariance[0] = 0.01; odom_msg.pose.covariance[7] = 0.01; odom_msg.pose.covariance[14] = 0.01;
  odom_msg.pose.covariance[21] = 0.01; odom_msg.pose.covariance[28] = 0.01; odom_msg.pose.covariance[35] = 0.01;

  // Publicar velocidad (opcional, se puede estimar a partir del cambio en la pose)
  odom_msg.twist.twist.linear.x = (d_left + d_right) / 2.0 / (odom_update_interval / 1000.0);
  odom_msg.twist.twist.angular.z = d_theta / (odom_update_interval / 1000.0);

  // Covarianza de la velocidad (ajusta según la confianza)
  odom_msg.twist.covariance[0] = 0.01; odom_msg.twist.covariance[7] = 0.01; odom_msg.twist.covariance[14] = 0.01;
  odom_msg.twist.covariance[21] = 0.01; odom_msg.twist.covariance[28] = 0.01; odom_msg.twist.covariance[35] = 0.01;

  odom_pub.publish(&odom_msg);

  // Publicar transformación TF
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q = tf::createQuaternionFromYaw(theta);
  transform.setRotation(q);
  odom_broadcaster.sendTransform(tf::StampedTransform(transform, nh.now(), "odom", "base_footprint"));
}