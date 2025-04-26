#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ArduinoHardware.h>

// Pines de control de motores (configuración modificada)
#define MOTOR_LEFT_PWM1 4
#define MOTOR_LEFT_PWM2 5
#define MOTOR_RIGHT_PWM1 6
#define MOTOR_RIGHT_PWM2 7

// Pines de encoder
#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 3
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 19

// Variables de encoder
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ROS NodeHandle
ros::NodeHandle nh;

// Publicador para el tópico /data_base
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);

std_msgs::Int32MultiArray encoder_msg;
ros::Publisher encoder_pub("/encoders", &encoder_msg);

// Suscriptor para el tópico cmd_vel
void velocity_cb(const geometry_msgs::Twist& twist_msg);
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velocity_cb);

// Variables de control de velocidad lineal y angular
float linear_x = 0;
float angular_z = 0;

unsigned long last_publish_time = 0;
const unsigned long publish_interval = 20; // 50Hz

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(encoder_pub);
  nh.subscribe(vel_sub);

  // Configurar pines de motor como salidas PWM
  pinMode(MOTOR_LEFT_PWM1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM2, OUTPUT);

  // Inicializar encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoder, CHANGE);

  // Inicializar IMU
  if(!bno.begin()) {
    Serial.println("Error al iniciar BNO055");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Arduino Mega - Nodo ROS Control de Motores");

  // Inicializar el array del encoder message
  encoder_msg.data = (int32_t*)malloc(sizeof(int32_t) * 2);
  encoder_msg.data_length = 2;
}

void loop() {
  // Publicar datos periódicamente
  if(millis() - last_publish_time >= publish_interval) {
    publishSensorData();
    last_publish_time = millis();
  }

  nh.spinOnce();
}

void publishSensorData() {
  // Leer IMU
  imu::Quaternion quat = bno.getQuat();
  imu::Vector linear_acceleration = bno.getLinearAccel();
  imu::Vector angular_velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Llenar el mensaje IMU
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_frame";

  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  // Puedes configurar la matriz de covarianza si tienes información sobre el ruido
  // Por ahora, la dejaremos en cero (desconocido)
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

  // Calcular ticks desde el último envío
  long delta_left = left_ticks - prev_left_ticks;
  long delta_right = right_ticks - prev_right_ticks;
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  // Llenar el mensaje del encoder
  encoder_msg.data[0] = delta_left;
  encoder_msg.data[1] = delta_right;
  encoder_pub.publish(&encoder_msg);
}

void velocity_cb(const geometry_msgs::Twist& twist_msg) {
  linear_x = twist_msg.linear.x;
  angular_z = twist_msg.angular.z;

  // Aquí puedes implementar la lógica para convertir las velocidades lineal y angular
  // en velocidades para cada motor. Esto dependerá de la geometría de tu robot (por ejemplo, diferencial).

  // Ejemplo para un robot diferencial simple:
  float wheel_radius = 0.05; // Radio de las ruedas en metros (ajusta según tu robot)
  float wheel_separation = 0.2; // Distancia entre las ruedas en metros (ajusta según tu robot)

  float left_vel = (linear_x - (angular_z * wheel_separation / 2.0)) / wheel_radius;
  float right_vel = (linear_x + (angular_z * wheel_separation / 2.0)) / wheel_radius;

  // Convertir velocidades (m/s) a valores PWM (aproximadamente proporcional)
  // Esto requerirá calibración para tu hardware específico.
  int left_pwm = map(left_vel * 100, -255, 255, -255, 255); // Factor de escala arbitrario
  int right_pwm = map(right_vel * 100, -255, 255, -255, 255); // Factor de escala arbitrario

  setMotorSpeed(MOTOR_LEFT_PWM1, MOTOR_LEFT_PWM2, left_pwm);
  setMotorSpeed(MOTOR_RIGHT_PWM1, MOTOR_RIGHT_PWM2, right_pwm);

  // Puedes imprimir las velocidades para depuración
  // Serial.print("Linear X: "); Serial.print(linear_x);
  // Serial.print(", Angular Z: "); Serial.print(angular_z);
  // Serial.print(", Left Vel: "); Serial.print(left_vel);
  // Serial.print(", Right Vel: "); Serial.println(right_vel);
}

void setMotorSpeed(int pwm_pin1, int pwm_pin2, int speed) {
  // Limitar velocidad entre -255 y 255
  speed = constrain(speed, -255, 255);

  if(speed > 0) {
    // Movimiento hacia adelante: PWM1 = velocidad, PWM2 = 0
    analogWrite(pwm_pin1, speed);
    analogWrite(pwm_pin2, 0);
  }
  else if(speed < 0) {
    // Movimiento hacia atrás: PWM1 = 0, PWM2 = |velocidad|
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, -speed); // Usamos -speed porque speed es negativo
  }
  else {
    // Detener motor: ambos PWM = 0
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, 0);
  }
}

// Interrupciones para encoders (sin cambios)
void handleLeftEncoder() {
  if(digitalRead(ENCODER_LEFT_A)) {
    left_ticks += (digitalRead(ENCODER_LEFT_B)) ? -1 : 1;
  } else {
    left_ticks += (digitalRead(ENCODER_LEFT_B)) ? 1 : -1;
  }
}

void handleRightEncoder() {
  if(digitalRead(ENCODER_RIGHT_A)) {
    right_ticks += (digitalRead(ENCODER_RIGHT_B)) ? 1 : -1;
  } else {
    right_ticks += (digitalRead(ENCODER_RIGHT_B)) ? -1 : 1;
  }
}