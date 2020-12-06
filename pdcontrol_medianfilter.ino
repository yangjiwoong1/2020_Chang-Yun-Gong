#include <Servo.h>
#define PIN_SERVO 10
#define PIN_LED 9
#define PIN_IR A0

#define DIST_TARGET 255
#define DIST_MIN 100
#define DIST_MAX 410

#define DUTY_MIN 1080
#define DUTY_NEU 1400
#define DUTY_MAX 1750

#define SERVO_ANGLE 30
#define SERVO_SPEED 120

#define INTERVAL_DIST 80
#define INTERVAL_SERVO 40
#define INTERVAL_SERIAL 25


#define DIST_ALPHA 0.6

#define _dist_array_length 3


#define _KP 2
#define _KD 20

Servo myservo;

int index = 0;
float dist_array[_dist_array_length];
float dist_array_copy[_dist_array_length];

float dist_target = DIST_TARGET;
float dist_raw,dist_median,dist_ema;

unsigned long last_sampling_time_dist,last_sampling_time_servo,last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED,OUTPUT);

  duty_target = duty_curr = DUTY_NEU;
  myservo.writeMicroseconds(DUTY_NEU);

  dist_array[_dist_array_length-1] = -1;
  dist_raw = 0;

  delay(500);
  //while(1){}

  Serial.begin(57600);
  duty_chg_per_interval = (DUTY_MAX-DUTY_MIN) * (SERVO_SPEED / 45.0) *(INTERVAL_SERVO/1000.0);
  //(duty_target-duty_curr//microseconds)*(servo_speed/duty gap-->angle conversion)*(interval/1000.0)
  last_sampling_time_dist = 500;
  last_sampling_time_servo = 500; 
  last_sampling_time_serial = 500;
    
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
    dist_raw = ir_distance();

  //median code
  if (index == _dist_array_length-1 && dist_array[index] != -1){
    for (int temp_index = 0;temp_index < index;temp_index++){
      dist_array[temp_index] = dist_array[temp_index+1];
    }
    dist_array[index] = dist_raw;
    Copy();
    Ascending_sort();
  
    if (_dist_array_length%2 == 1) dist_median = dist_array_copy[(index/2)];
     else {
      int temp = (index-1)/2 ;
      dist_median = (dist_array_copy[temp]+dist_array_copy[temp+1])/2 ;
    }
    }
    
  else if (index == _dist_array_length-1 && dist_array[index] == -1){
    dist_array[index] = dist_raw;
    Copy();
    Ascending_sort();
    if (_dist_array_length%2 == 1) dist_median = dist_array_copy[(index/2)];
     else {
      int temp = (index-1)/2 ;
      dist_median = (dist_array_copy[temp]+dist_array_copy[temp+1])/2 ;
    }
  }

  else {    
    dist_array[index] = dist_raw;
    dist_median = 0;
    index += 1;
  }
  
    //after sampling
    if (index == _dist_array_length-1 && dist_array[index] != -1) {

    dist_raw = dist_median;
    dist_ema = DIST_ALPHA * dist_raw + (1-DIST_ALPHA) * dist_ema;
    dist_raw = dist_ema;

    error_curr = dist_target - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm; // pterm + dterm + iterm
    duty_target = DUTY_NEU + control;

    if(duty_target < DUTY_MIN) duty_target = DUTY_MIN;
    if(duty_target > DUTY_MAX) duty_target = DUTY_MAX;
    
    error_prev = error_curr;
    }
    
    last_sampling_time_dist += INTERVAL_DIST;
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
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    //Serial.print("error_curr:");
    //Serial.print(error_curr);
    //Serial.print("error_prev:");
    //Serial.print(error_prev);
    //Serial.print("difference:");
    //Serial.print(error_curr - error_prev);
    
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));   
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    last_sampling_time_serial += INTERVAL_SERIAL;
  }
}

float ir_distance(void){
  float val;
  float cali;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  
  if (val < 181.0)   cali = 100 + 100.0 / (181 - 71) * (val - 71);
  else if (val < 220.0)   cali = 200 + 55.0 / (220 - 181) * (val - 181);
  else if (val < 234.0)   cali = 255 + 45.0 / (234 - 220) * (val - 220);
  else cali = 300 + 110.0 / (306 - 234) * (val - 234);


  return cali>420?410:cali;
}


void Copy() {
  for (int i=0;i<=_dist_array_length - 1;i++)
  dist_array_copy[i] = dist_array[i];
}

void Ascending_sort(){
 float temp;
 for (int i=0;i<_dist_array_length-1;i++){
  for (int k=0;k<_dist_array_length-1;k++) {
  if (dist_array_copy[k]>dist_array_copy[k+1]){
    temp = dist_array_copy[k];
    dist_array_copy[k] = dist_array_copy[k+1];
    dist_array_copy[k+1] = temp;
  }
 }

}
}
