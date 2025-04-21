#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>
//Define los pines de los sensores ultrasonicos
#define ECHO_PIN_1 22   //trasero der
#define TRIG_PIN_1 23
#define ECHO_PIN_2 24   // trasero izq
#define TRIG_PIN_2 25
#define ECHO_PIN_3 26   //frontal der
#define TRIG_PIN_3 27
#define ECHO_PIN_4 28   //frontal izq
#define TRIG_PIN_4 29
//#define ECHO_PIN_5 31
//#define TRIG_PIN_5 30
// Definir los canales del PCA9685 para cada servo
#define PAN_FRONT_SERVO 0
#define TILT_FRONT_SERVO 1
#define PAN_REVERSE_SERVO 2
#define TILT_REVERSE_SERVO 3
#define TRAY_LEFT_SERVO 4
#define TRAY_RIGHT_SERVO 5

// Definición de los pines del Arduino conectados al VNH2SP30
const int inAPin = 10;  // INA pin
const int inBPin = 9;  // INB pin
const int pwMPin = 8;  // PWM pin

// Definición de pines para los motores
const int motorRightForward = 4;
const int motorRightBackward = 5;
const int motorLeftForward = 6;
const int motorLeftBackward = 7;
// Inicializar el controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long previousMillis = 0;
const long interval = 100; // Intervalo de tiempo en milisegundos
// Inicializar el (IMU)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(9600);
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  Serial.println("Controlador de servo PCA9685");
  pwm.begin();
  pwm.setPWMFreq(60);  // Frecuencia típica para servos
  // Coloca los servos en la posición inicial
  resetServos();
  //Configurando pines de servos
   pinMode(TRIG_PIN_1, OUTPUT);
   pinMode(ECHO_PIN_1, INPUT);
   pinMode(TRIG_PIN_2, OUTPUT);
   pinMode(ECHO_PIN_2, INPUT);
   pinMode(TRIG_PIN_3, OUTPUT);
   pinMode(ECHO_PIN_3, INPUT);
   pinMode(TRIG_PIN_4, OUTPUT);
   pinMode(ECHO_PIN_4, INPUT);

  //Configuración de motores
  pinMode(inAPin, OUTPUT);
  pinMode(inBPin, OUTPUT);
  pinMode(pwMPin, OUTPUT);
  // Inicializa los pines de motor como salidas
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);

  // Inicia los motores en estado detenido
  stopMotors();
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
}
void loop(void)
{
  unsigned long currentMillis = millis();
  sensors_event_t event;
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        readUltrasonicSensors();
        bno.getEvent(&event);
        Serial.println(event.orientation.x, 4);
    }
  sweepCameraFront();
  sweepCameraReverse();
  //unloadTray();
  //resetServos();
  //delay(5000);  // Espera un segundo antes de reiniciar el ciclo
  turnLeftBothMotors();
  delay(3000);
  advance();
  activaBarredora();
  delay(10000);
  back();
  //desactivaBarredora();
  delay(2000);
  desactivaBarredora();
  //turnRightOneMotor();
  //delay(3000);
  turnRightBothMotors();
  delay(6000);
  stopMotors();
  back();
  //desactivaBarredora();
    delay(3000);
  //unloadTray();
  resetServos();
  //delay(5000);
}

void readUltrasonicSensors() {
    Serial.print(getDistance(TRIG_PIN_1, ECHO_PIN_1));     Serial.print(",");
    Serial.print(getDistance(TRIG_PIN_2, ECHO_PIN_2));     Serial.print(",");
    Serial.print(getDistance(TRIG_PIN_3, ECHO_PIN_3));     Serial.print(",");
    Serial.print(getDistance(TRIG_PIN_4, ECHO_PIN_4));     Serial.print(",");
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // Velocidad del sonido en cm/ms
    return distance;
}

void resetServos() {
  setServo(PAN_FRONT_SERVO, 80);//der 10 -  80   -  150  izq mirando hacia el frente
  setServo(TILT_FRONT_SERVO, 115);//arriba 160  -  115 -  40  abajo
  setServo(PAN_REVERSE_SERVO, 85); //izq 160 - 85 -  10 der mirando hacia atras
  setServo(TILT_REVERSE_SERVO, 70); // arriba 10 - 70 -  130 abajo
  setServo(TRAY_LEFT_SERVO, 0);  // servo izquierdo cerrado en 0 - 130 
  setServo(TRAY_RIGHT_SERVO, 176); // servo derecho cerrado en 176 - 0 
}

void sweepCameraFront() {
  for (int angle = 0; angle <= 180; angle += 10) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(300);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    setServo(PAN_FRONT_SERVO, angle);
    delay(300);
  }
}

void sweepCameraReverse() {
  for (int angle = 0; angle <= 180; angle += 10) {
    setServo(PAN_REVERSE_SERVO, angle);
    delay(100);
  }
  for (int angle = 180; angle >= 0; angle -= 10) {
    setServo(PAN_REVERSE_SERVO, angle);
    delay(100);
  }
}

void unloadTray() {
  // Suponiendo que los servos están invertidos para la bandeja
  for (int angle = 0; angle <= 90; angle += 1) {
    setServo(TRAY_LEFT_SERVO, map(angle,0,180,0,130));  
    setServo(TRAY_RIGHT_SERVO, 176-angle);//Invertido
    delay(50);
  }
  // Regresar la bandeja a su posición inicial
  for (int angle = 90; angle >= 0; angle -= 1) {
    setServo(TRAY_LEFT_SERVO, map(angle,0,180,0,130));  
    setServo(TRAY_RIGHT_SERVO, 176-angle);//Invertido
    delay(50);
  }
  setServo(TRAY_LEFT_SERVO, 0);
  setServo(TRAY_RIGHT_SERVO, 176);
}

void setServo(int channel, int angle) {
  int pulse = map(angle, 0, 180, 125, 625);  // Mapea el ángulo a pulsos (ajuste según tu servo)
  pwm.setPWM(channel, 0, pulse);
}

void activaBarredora(){
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, HIGH);
  analogWrite(pwMPin, 255);
}
void desactivaBarredora(){
  digitalWrite(inAPin, LOW);
  digitalWrite(inBPin, LOW);
  analogWrite(pwMPin, 0); 
}

void advance() {
  analogWrite(motorRightForward, 255);
  analogWrite(motorLeftForward, 255);
  stopBackward();
}

void back() {
  analogWrite(motorRightBackward, 255);
  analogWrite(motorLeftBackward, 255);
  stopForward();
}

void turnLeftOneMotor() {
  analogWrite(motorRightForward, 255);
  stopMotorsLeft();
}

void turnRightOneMotor() {
  analogWrite(motorLeftForward, 255);
  stopMotorsRight();
}

void turnLeftBothMotors() {
  analogWrite(motorRightForward, 255);
  analogWrite(motorLeftBackward, 255);
  stopBackwardRight();
}

void turnRightBothMotors() {
  analogWrite(motorRightBackward, 255);
  analogWrite(motorLeftForward, 255);
  stopForwardRight();
}

void stopMotors() {
  stopForward();
  stopBackward();
}

void stopForward() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward, 0);
}

void stopBackward() {
  analogWrite(motorRightBackward, 0);
  analogWrite(motorLeftBackward, 0);
}

void stopMotorsRight() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorRightBackward, 0);
}

void stopMotorsLeft() {
  analogWrite(motorLeftForward, 0);
  analogWrite(motorLeftBackward, 0);
}

void stopBackwardRight() {
  analogWrite(motorRightBackward, 0);
  analogWrite(motorLeftForward, 0);
}

void stopForwardRight() {
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftBackward, 0);
}
