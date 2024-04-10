const int pinnum = 8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

bool checktime(int pin, int t){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(1);
  pinMode(pin, INPUT);
  delay(t);
  return (digitalRead(pin)==HIGH);
}

int findtime(int pin, int maxt){
  // check how large the value is using binary search, return maxt if t >= maxt
  int tmax=maxt;
  int tmin=0;
  int tmid;
  while (tmax>tmin){
    tmid = (tmax+tmin)/2;
    if (checktime(pin, tmid)){
      tmin=tmid+1;
    } else{
      tmax = tmid;
    }
  }
  return tmax;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(findtime(pinnum, 31));
}
