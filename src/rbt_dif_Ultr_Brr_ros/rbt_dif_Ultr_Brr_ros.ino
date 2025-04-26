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
#include <std_msgs/Bool.h> // Para el control de la barredora
#include <ArduinoHardware.h>

// Definiciones para el Control de Motores ---
#define MOTOR_LEFT_PWM1 4
#define MOTOR_LEFT_PWM2 5
#define MOTOR_RIGHT_PWM1 6
#define MOTOR_RIGHT_PWM2 7

// Definiciones para Encoders ---
#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 3
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 19

// Definiciones para Sensores Ultrasónicos ---
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

// Definiciones para la Barredora (VNH2SP30) ---
const int inAPin = 10;  // INA pin
const int inBPin = 9;   // INB pin
const int pwMPin = 8;   // PWM pin

// Variables para Encoders ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;

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
const long intervalUltrasonic = 50; // Intervalo de 50ms (20Hz)

// --- Objetos NewPing para sensores ultrasónicos ---
NewPing sonar_front_left(FRONT_LEFT_TRIG, FRONT_LEFT_ECHO, 200);
NewPing sonar_front_right(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO, 200);
NewPing sonar_rear_left(REAR_LEFT_TRIG, REAR_LEFT_ECHO, 200);
NewPing sonar_rear_right(REAR_RIGHT_TRIG, REAR_RIGHT_ECHO, 200);
NewPing sonar_rear_low(REAR_LOW_TRIG, REAR_LOW_ECHO, 200);

// --- Funciones de configuración para mensajes Range ---
void setupRangeMessage(sensor_msgs::Range &range_msg, const char *frame_id);
void readAndPublishUltrasonic(NewPing &sonar, sensor_msgs::Range &range_msg, ros::Publisher &publisher);

// --- Funciones para la barredora ---
void activaBarredora();
void desactivaBarredora();

unsigned long last_publish_time = 0;
const unsigned long publish_interval = 20; // 50Hz

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
desactivaBarredora(); // Asegurar que la barredora comience desactivada

Serial.println("Arduino Mega - Nodo ROS Integrado");
}
void publishSensorData();
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

nh.spinOnce();
delayMicroseconds(100);
}

void publishSensorData() {
// --- Leer IMU ---
imu::Quaternion quat = bno.getQuat();
imu::Vector(float) linear_acceleration = bno.getLinearAccel();
imu::Vector(float) angular_velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

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
float wheel_radius = 0.05;
float wheel_separation = 0.2;
float left_vel = (linear_x - (angular_z * wheel_separation / 2.0)) / wheel_radius;
float right_vel = (linear_x + (angular_z * wheel_separation / 2.0)) / wheel_radius;
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
range_msg.field_of_view = 0.26; // ~15 grados
range_msg.min_range = 0.02; // 2cm
range_msg.max_range = 2.0;  // 2m
}

void readAndPublishUltrasonic(NewPing &sonar, sensor_msgs::Range &range_msg, ros::Publisher &publisher) {
unsigned int cm = sonar.ping_cm();
range_msg.range = cm / 100.0; // Convertir a metros
range_msg.header.stamp = nh.now();
publisher.publish(&range_msg);
}

void activaBarredora() {
digitalWrite(inAPin, LOW);
digitalWrite(inBPin, HIGH);
analogWrite(pwMPin, 255); // Ajustar velocidad PWM si es necesario
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
