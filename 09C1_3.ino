// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _dist_array_length 3

// global variables
int index = 0;
float dist_array[_dist_array_length];
float dist_array_copy[_dist_array_length];
float timeout; // unit: us
float dist_min, dist_max, dist_raw,dist_median; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  dist_array[_dist_array_length-1] = -1;
// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
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

 
// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("midian:");
  //Serial.print(dist_median);
  Serial.print(map(dist_median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
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
