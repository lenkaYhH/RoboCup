int pin=8;
int time;
// test
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
int infsensor(int infpin) {
  // return the amount of delay of the infrared sensor
  // larger value = more black
  int t=0;
  pinMode(infpin, OUTPUT);
  digitalWrite(infpin, HIGH);
  delay(1);
  pinMode(infpin,INPUT);
  while(digitalRead(infpin)==HIGH){
    t= t+1;
    delayMicroseconds(100);
  }
  return t;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  time = infsensor(pin);
  Serial.println(time);
}
