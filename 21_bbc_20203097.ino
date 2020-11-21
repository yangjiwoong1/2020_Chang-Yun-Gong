#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

Servo myservo;


int a, b; // unit: mm
float filtered;
float sensitivity = 0.1;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1400);
  
// initialize serial port
  Serial.begin(57600);

  a = 70;
  b = 320;
  filtered = float(analogRead(PIN_IR));
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  filtered = filtered * (1-sensitivity) + volt * sensitivity;
  val = ((6762.0/(filtered-9.0))-4.0) * 10.0;
  return val;
}


void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > 255) myservo.writeMicroseconds(1080);
  else myservo.writeMicroseconds(1750);
  delay(100);
}
