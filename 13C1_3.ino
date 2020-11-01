#include <Servo.h>

#define PIN_SERVO 10
#define _DUTY_MIN 553
#define _DUTY_MAX 2399
#define _DUTY_NEU 1476
#define _SERVO_SPEED 3

unsigned long last_sampling_time;
int duty_chg_per_interval;
Servo myservo;
int duty_target, duty_curr;
int interval;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  duty_target = _DUTY_MAX;
  duty_curr = _DUTY_MIN;
  duty_chg_per_interval = 1;
  interval = (180.0/(_DUTY_MAX - _DUTY_MIN))*(1000.0/_SERVO_SPEED)*duty_chg_per_interval;
  myservo.writeMicroseconds(_DUTY_MIN);
  last_sampling_time = 800;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() < last_sampling_time + interval) return;
  if (duty_curr < duty_target) {
      duty_curr += duty_chg_per_interval;
      myservo.writeMicroseconds(duty_curr);
      last_sampling_time += interval;
  }
}
