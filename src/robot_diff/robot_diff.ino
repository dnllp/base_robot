#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Pines de control de motores (configuración modificada)
#define MOTOR_LEFT_PWM1 4    // Pin PWM1 motor izquierdo (velocidad positiva)
#define MOTOR_LEFT_PWM2 5    // Pin PWM2 motor izquierdo (velocidad negativa)
#define MOTOR_RIGHT_PWM1 6   // Pin PWM1 motor derecho (velocidad positiva)
#define MOTOR_RIGHT_PWM2 7   // Pin PWM2 motor derecho (velocidad negativa)

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

// Variables de control
int left_speed = 0;
int right_speed = 0;
unsigned long last_send_time = 0;
const unsigned long send_interval = 20; // 50Hz (20ms)

void setup() {
  Serial.begin(57600);
  
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
  
  Serial.println("Arduino Mega - Control de Motores Modificado");
}

void loop() {
  // Enviar datos periódicamente
  if(millis() - last_send_time >= send_interval) {
    sendSensorData();
    last_send_time = millis();
  }
  
  // Procesar comandos de ROS
  if(Serial.available()) {
    processROSCommand();
  }
}

void sendSensorData() {
  // Leer IMU
  imu::Quaternion quat = bno.getQuat();
  
  // Calcular ticks desde el último envío
  long delta_left = left_ticks - prev_left_ticks;
  long delta_right = right_ticks - prev_right_ticks;
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;
  
  // Enviar datos por serial
  Serial.print("E");
  Serial.print(delta_left);
  Serial.print(",");
  Serial.print(delta_right);
  Serial.print(" I");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");
  Serial.print(quat.w(), 4);
  Serial.println();
}

void processROSCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  // Procesar comando de velocidad (formato: "L<vel> R<vel>")
  if(command.startsWith("L") && command.indexOf("R") > 0) {
    int l_pos = command.indexOf('L');
    int r_pos = command.indexOf('R');
    int space_pos = command.indexOf(' ');
    
    if(l_pos >= 0 && r_pos > l_pos && space_pos > l_pos) {
      left_speed = command.substring(l_pos+1, space_pos).toInt();
      right_speed = command.substring(r_pos+1).toInt();
      
      // Controlar motores con el nuevo esquema
      setMotorSpeed(MOTOR_LEFT_PWM1, MOTOR_LEFT_PWM2, left_speed);
      setMotorSpeed(MOTOR_RIGHT_PWM1, MOTOR_RIGHT_PWM2, right_speed);
    }
  }
}

// Función modificada según tus especificaciones
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
