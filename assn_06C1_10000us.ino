#define LED 7
int period;
double on_time;
double off_time;


void setup() {
  // put your setup code here, to run once:
pinMode(LED,OUTPUT);
period=set_period(10000);
on_time,off_time=0;
}

void loop() {
 set_duty();
}


unsigned int set_period(unsigned int period){
  return period;
}


void set_duty(){
  for (int duty = 0 ; duty <= 100; duty+=1)  {
 on_time = (period/100)*duty;
 off_time = period - on_time;
  digitalWrite(LED,LOW);
  delayMicroseconds(on_time);
  digitalWrite(LED,HIGH);
  delayMicroseconds(off_time);
}
for (int duty = 100 ; duty >= 0; duty -= 1) {
 on_time = (period/100)*duty;
 off_time = period - on_time;
  digitalWrite(LED,LOW);
  delayMicroseconds(on_time);
  digitalWrite(LED,HIGH);
  delayMicroseconds(off_time);
 }
}

 
