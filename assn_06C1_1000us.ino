#define LED 7
int period;
double on_time;
double off_time;


void setup() {
  // put your setup code here, to run once:
pinMode(LED,OUTPUT);
period=set_period(1000);
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
 for (int a = 0; a<4; a+=1){
  digitalWrite(LED,LOW);
  delayMicroseconds(on_time);
  digitalWrite(LED,HIGH);
  delayMicroseconds(off_time);
  }
delay(1);
}
for (int duty = 100 ; duty >= 0; duty -= 1) {
 on_time = (period/100)*duty;
 off_time = period - on_time;
 for (int a = 0; a<4; a+=1){
  digitalWrite(LED,LOW);
  delayMicroseconds(on_time);
  digitalWrite(LED,HIGH);
  delayMicroseconds(off_time);
  }
 delay(1);
 }
}

 
