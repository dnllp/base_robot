#include <NewPing.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h> // Incluimos String para un posible mensaje de debug

// Definición de pines para sensores ultrasónicos
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

// Configuración de ROS
ros::NodeHandle nh;

// Mensajes para cada sensor
sensor_msgs::Range front_left_range;
sensor_msgs::Range front_right_range;
sensor_msgs::Range rear_left_range;
sensor_msgs::Range rear_right_range;
sensor_msgs::Range rear_low_range;

// Publicadores
ros::Publisher pub_front_left("/ultrasonic/front_left", &front_left_range);
ros::Publisher pub_front_right("/ultrasonic/front_right", &front_right_range);
ros::Publisher pub_rear_left("/ultrasonic/rear_left", &rear_left_range);
ros::Publisher pub_rear_right("/ultrasonic/rear_right", &rear_right_range);
ros::Publisher pub_rear_low("/distancia", &rear_low_range);

// Objetos NewPing para cada sensor
NewPing sonar_front_left(FRONT_LEFT_TRIG, FRONT_LEFT_ECHO, 200); // Distancia máxima 200cm
NewPing sonar_front_right(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO, 200);
NewPing sonar_rear_left(REAR_LEFT_TRIG, REAR_LEFT_ECHO, 200);
NewPing sonar_rear_right(REAR_RIGHT_TRIG, REAR_RIGHT_ECHO, 200);
NewPing sonar_rear_low(REAR_LOW_TRIG, REAR_LOW_ECHO, 200);

// Variables para temporización no bloqueante
unsigned long previousMillis = 0;
const long interval = 50;  // Intervalo de 50ms (20Hz)

void setup() {
  // Inicializar ROS
  nh.initNode();
  nh.advertise(pub_front_left);
  nh.advertise(pub_front_right);
  nh.advertise(pub_rear_left);
  nh.advertise(pub_rear_right);
  nh.advertise(pub_rear_low);

  // Configurar mensajes Range con valores iniciales (opcional pero útil)
  setupRangeMessage(front_left_range, "front_left");
  setupRangeMessage(front_right_range, "front_right");
  setupRangeMessage(rear_left_range, "rear_left");
  setupRangeMessage(rear_right_range, "rear_right");
  setupRangeMessage(rear_low_range, "rear_low");

  //Serial.begin(115200); // Inicializar Serial para debugging
  //Serial.println("Nodo ROS de ultrasonidos inicializado.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Lectura y publicación de sensores ultrasónicos cada 50ms
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Actualizar el tiempo anterior

    // Leer y publicar datos de cada sensor
    readAndPublish(sonar_front_left, front_left_range, pub_front_left);
    readAndPublish(sonar_front_right, front_right_range, pub_front_right);
    readAndPublish(sonar_rear_left, rear_left_range, pub_rear_left);
    readAndPublish(sonar_rear_right, rear_right_range, pub_rear_right);
    readAndPublish(sonar_rear_low, rear_low_range, pub_rear_low);
  }

  nh.spinOnce();
  delayMicroseconds(100);
}

void setupRangeMessage(sensor_msgs::Range &range_msg, const char *frame_id) {
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = frame_id;
  range_msg.field_of_view = 0.26; // ~15 grados
  range_msg.min_range = 0.02;     // 2cm
  range_msg.max_range = 2.0;      // 2m
}

void readAndPublish(NewPing &sonar, sensor_msgs::Range &range_msg, ros::Publisher &publisher) {
  unsigned int cm = sonar.ping_cm();
  range_msg.range = cm / 100.0; // Convertir a metros
  range_msg.header.stamp = nh.now();
  publisher.publish(&range_msg);
  //Serial.print("Publicando en ");
  //Serial.print(publisher.getTopic());
 // Serial.print(": ");
  //Serial.println(range_msg.range);
}
