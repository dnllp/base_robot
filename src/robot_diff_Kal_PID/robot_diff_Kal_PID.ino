#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Pines de control de motores
#define MOTOR_LEFT_PWM1 4    // PWM para dirección positiva
#define MOTOR_LEFT_PWM2 5    // PWM para dirección negativa
#define MOTOR_RIGHT_PWM1 6   // PWM para dirección positiva
#define MOTOR_RIGHT_PWM2 7   // PWM para dirección negativa

// Pines de encoder
#define ENCODER_LEFT_A 18
#define ENCODER_LEFT_B 19
#define ENCODER_RIGHT_A 20
#define ENCODER_RIGHT_B 21

// Parámetros del robot
const float WHEEL_RADIUS = 0.065;    // Radio de rueda en metros
const float WHEEL_BASE = 0.30;       // Distancia entre ruedas en metros
const int ENCODER_TICKS = 980;       // Ticks por revolución del encoder

// Variables de encoder
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Estructura para control PID
typedef struct {
  float Kp, Ki, Kd;      // Ganancias
  float integral;         // Acumulador integral
  float prev_error;       // Error anterior
  unsigned long last_time;// Última actualización
} PIDController;

// Controladores PID para cada motor
PIDController pid_left, pid_right;

// Variables para el Filtro de Kalman
typedef struct {
  float x, y, theta;     // Posición y orientación
  float P[3][3];         // Matriz de covarianza
} KalmanState;

KalmanState kstate;

// Variables de control
float target_left_speed = 0;   // Velocidad objetivo (m/s)
float target_right_speed = 0;
float current_left_speed = 0;
float current_right_speed = 0;
unsigned long last_send_time = 0;
const unsigned long send_interval = 20; // 50Hz (20ms)
unsigned long last_update_time = 0;

void setup() {
  Serial.begin(57600);
  
  // Configurar pines de motor
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
  
  // Inicializar controladores PID
  initPIDController(&pid_left, 2.0, 5.0, 0.1);  // Kp, Ki, Kd
  initPIDController(&pid_right, 2.0, 5.0, 0.1);
  
  // Inicializar Filtro de Kalman
  initKalmanFilter();
  
  Serial.println("Sistema listo - PID activado");
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - last_update_time) / 1000.0; // en segundos
  
  if(dt >= 0.02) {  // Ejecutar a 50Hz
    // 1. Calcular velocidades actuales
    current_left_speed = calculateSpeed(left_ticks, prev_left_ticks, dt);
    current_right_speed = calculateSpeed(right_ticks, prev_right_ticks, dt);
    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;
    
    // 2. Ejecutar control PID
    int left_pwm = computePID(&pid_left, target_left_speed, current_left_speed, dt);
    int right_pwm = computePID(&pid_right, target_right_speed, current_right_speed, dt);
    
    // 3. Aplicar señales PWM a los motores
    setMotorSpeed(MOTOR_LEFT_PWM1, MOTOR_LEFT_PWM2, left_pwm);
    setMotorSpeed(MOTOR_RIGHT_PWM1, MOTOR_RIGHT_PWM2, right_pwm);
    
    // 4. Actualizar Filtro de Kalman
    float encoder_theta = (current_right_speed - current_left_speed) * dt / WHEEL_BASE;
    float imu_theta = getIMUHeading();
    float imu_ang_vel = getIMUAngularVelocity();
    
    predictKalman(dt, imu_ang_vel);
    updateKalman(encoder_theta, imu_theta);
    
    last_update_time = current_time;
  }
  
  // Enviar datos periódicamente
  if(current_time - last_send_time >= send_interval) {
    sendTelemetry();
    last_send_time = current_time;
  }
  
  // Procesar comandos de ROS
  if(Serial.available()) {
    processROSCommand();
  }
}

// Inicialización del control PID
void initPIDController(PIDController* pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->integral = 0;
  pid->prev_error = 0;
  pid->last_time = millis();
}

// Cálculo del control PID
int computePID(PIDController* pid, float target, float current, float dt) {
  // Calcular error
  float error = target - current;
  
  // Término proporcional
  float P = pid->Kp * error;
  
  // Término integral (con anti-windup)
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -255/pid->Ki, 255/pid->Ki); // Limitar integral
  float I = pid->Ki * pid->integral;
  
  // Término derivativo
  float D = pid->Kd * (error - pid->prev_error) / dt;
  pid->prev_error = error;
  
  // Calcular salida del controlador
  float output = P + I + D;
  
  // Limitar salida a rango PWM
  return constrain((int)output, -255, 255);
}

// Inicialización del Filtro de Kalman
void initKalmanFilter() {
  kstate.x = 0.0;
  kstate.y = 0.0;
  kstate.theta = 0.0;
  
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      kstate.P[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
}

// Predicción del estado (modelo de movimiento)
void predictKalman(float dt, float angular_velocity) {
  // Modelo de movimiento
  float linear_velocity = (current_left_speed + current_right_speed) / 2.0;
  kstate.x += linear_velocity * cos(kstate.theta) * dt;
  kstate.y += linear_velocity * sin(kstate.theta) * dt;
  kstate.theta += angular_velocity * dt;
  
  // Normalizar ángulo
  while(kstate.theta > PI) kstate.theta -= 2*PI;
  while(kstate.theta < -PI) kstate.theta += 2*PI;
  
  // Actualizar matriz de covarianza (simplificado)
  for(int i=0; i<3; i++) {
    kstate.P[i][i] += 0.01; // Añadir incertidumbre del proceso
  }
}

// Actualización del estado con mediciones
void updateKalman(float encoder_theta, float imu_theta) {
  // Fusionar orientaciones con pesos basados en confianza
  float kalman_gain = 0.8; // Más peso a la IMU
  kstate.theta = kalman_gain * imu_theta + (1-kalman_gain) * (kstate.theta + encoder_theta);
  
  // Normalizar ángulo
  while(kstate.theta > PI) kstate.theta -= 2*PI;
  while(kstate.theta < -PI) kstate.theta += 2*PI;
}

// Calcular velocidad actual en m/s
float calculateSpeed(long current_ticks, long prev_ticks, float dt) {
  if(dt <= 0) return 0;
  float distance = (2 * PI * WHEEL_RADIUS * (current_ticks - prev_ticks)) / ENCODER_TICKS;
  return distance / dt;
}

// Obtener orientación de la IMU
float getIMUHeading() {
  imu::Quaternion quat = bno.getQuat();
  float q0 = quat.w(), q1 = quat.x(), q2 = quat.y(), q3 = quat.z();
  return atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}

// Obtener velocidad angular de la IMU
float getIMUAngularVelocity() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return gyro.z(); // rad/s
}

// Control de motores con PWM independiente
void setMotorSpeed(int pwm_pin1, int pwm_pin2, int speed) {
  speed = constrain(speed, -255, 255);
  
  if(speed > 0) {
    analogWrite(pwm_pin1, speed);
    analogWrite(pwm_pin2, 0);
  } 
  else if(speed < 0) {
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, -speed);
  } 
  else {
    analogWrite(pwm_pin1, 0);
    analogWrite(pwm_pin2, 0);
  }
}

// Enviar datos de telemetría
void sendTelemetry() {
  Serial.print("TELEM ");
  Serial.print(target_left_speed, 3); Serial.print(" ");
  Serial.print(current_left_speed, 3); Serial.print(" ");
  Serial.print(target_right_speed, 3); Serial.print(" ");
  Serial.print(current_right_speed, 3); Serial.print(" ");
  Serial.print(kstate.x, 3); Serial.print(" ");
  Serial.print(kstate.y, 3); Serial.print(" ");
  Serial.print(kstate.theta, 3);
  Serial.println();
}

// Procesar comandos de velocidad desde ROS
void processROSCommand() {
  if(Serial.available() >= 12) { // Asumiendo formato "L1.23 R1.23"
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command.startsWith("L") && command.indexOf("R") > 0) {
      int l_pos = command.indexOf('L');
      int r_pos = command.indexOf('R');
      int space_pos = command.indexOf(' ');
      
      if(l_pos >= 0 && r_pos > l_pos && space_pos > l_pos) {
        target_left_speed = command.substring(l_pos+1, space_pos).toFloat();
        target_right_speed = command.substring(r_pos+1).toFloat();
      }
    }
  }
}

// Interrupciones para encoders
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