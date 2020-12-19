#include <Servo.h>
#define PIN_SERVO 10
#define PIN_LED 9
#define PIN_IR A0

#define DIST_TARGET 255
#define DIST_MIN 100
#define DIST_MAX 410

#define DIST_ALPHA 0.35

#define DUTY_MIN 1170
#define DUTY_NEU 1450
#define DUTY_MAX 1780

#define SERVO_ANGLE 30.0
#define SERVO_SPEED 200

#define DELAY_MICROS 1500

#define INTERVAL_DIST 30
#define INTERVAL_SERVO 40
#define INTERVAL_SERIAL 100

#define _KP 1.7 //  normal : 1.7,50,0.01  0.35
#define _KD 60
#define _KI 0.02

Servo myservo;


float filtered_dist = 0;
int samples_num = 3;

float dist_target = DIST_TARGET;
float dist_raw,dist_ema,x= 0;

unsigned long last_sampling_time_dist,last_sampling_time_servo,last_sampling_time_serial;

bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr = 0;

float error_curr, control, pterm, dterm = 0;
float iterm;
float error_prev = 155;

const float coE[] = {-0.0000293, 0.0176077, -1.8786798, 164.2327593};


void setup() {
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED,OUTPUT);

  duty_target = duty_curr = DUTY_NEU;
  myservo.writeMicroseconds(DUTY_NEU);

  iterm = 0;

  delay(5000);
  //while(1){}

  Serial.begin(57600);
  duty_chg_per_interval = (DUTY_MAX-DUTY_MIN) * (SERVO_SPEED / SERVO_ANGLE) *(INTERVAL_SERVO/1000.0);
  
  last_sampling_time_dist = 5000;
  last_sampling_time_servo = 5000; 
  last_sampling_time_serial = 5000;
    
  event_dist = false;
  event_servo = false;
  event_serial = false;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() >= last_sampling_time_dist + INTERVAL_DIST) event_dist= true;
  if (millis() >= last_sampling_time_servo + INTERVAL_SERVO) event_servo= true;
  if (millis() >= last_sampling_time_serial + INTERVAL_SERIAL) event_serial= true;

  if(event_dist){
    event_dist = false;
    
    x = filtered_ir_distance();
    filtered_dist = coE[0] * pow(x,3) + coE[1] * pow(x,2) + coE[2] * x + coE[3]; 
       
    error_curr = dist_target - filtered_dist ;
   
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm; //pterm + dterm + iterm
    duty_target = DUTY_NEU + control;

    if(duty_target < DUTY_MIN) duty_target = DUTY_MIN;
    if(duty_target > DUTY_MAX) duty_target = DUTY_MAX;
    
    last_sampling_time_dist += INTERVAL_DIST;
    error_prev = error_curr;
  }

  if(event_servo){
    event_servo = false;
    
    if(duty_target > duty_curr){
      duty_curr += duty_chg_per_interval;
      if(duty_curr >= duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

   myservo.writeMicroseconds(duty_curr);
   
   last_sampling_time_servo += INTERVAL_SERVO; 
  }
  

  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(filtered_dist);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

    last_sampling_time_serial += INTERVAL_SERIAL;
  }
}


float ir_distance(void){
  float val;
  float dist_cali;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;

  return val;
}


float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }

    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // ema 필터 추가
  dist_ema = DIST_ALPHA*lowestReading + (1-DIST_ALPHA)*dist_ema;
  return dist_ema;
}
