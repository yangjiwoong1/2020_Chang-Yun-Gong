#define PIN_LED 7
int temp;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,0);
  delay(1000);
  temp = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  while(temp<5) {
  digitalWrite(PIN_LED,1);
  delay(100);
  digitalWrite(PIN_LED,0);
  delay(100);
  temp += 1;
}
  while(1) {
  digitalWrite(PIN_LED,1);
  }
}
