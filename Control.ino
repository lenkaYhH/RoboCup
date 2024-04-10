// VARIABLES DEFINITION ----------------------------
// LINK to schematic https://www.johnval.nl/school/informatica/arduino/

// 2, 3, 9 is the left motor
const int controlPin1 = 2; // aangesloten aan pin 7 van de H-bridge
const int controlPin2 = 3; // aangesloten aan pin 2 van de H-bridge
const int enablePin = 9;   // aangesloten aan pin 1 van de H-bridge

// 6, 7, 10 is the right motor
const int controlPin12 = 6; // aangesloten aan pin 15 van de H-bridge
const int controlPin22 = 7; // aangesloten aan pin 10 van de H-bridge
const int enablePin2 = 10;   // aangesloten aan pin 9 van de H-bridge

// Ultrasonic
const int trigPin =  4;
const int echoPin =  5; 
long duration;
float dist;

// Infrared pins
// const int rightIFpin = 12;
// const int leftIFpin = 13;
// int right, left;
// left to right order
const int IFpins[5] = [12, 13, 14, 15, 16]

// const int thresholdL = 15;
const int thresholdR = 3;

const int forwardTimeout = 100; // in ms
const int forwardTime = 100;
const int turnTimeout = 100;
const int turnTime = 80;
// const int backTime = 80;
const int backTime = 200;

// other default values
int timeout = 50; // in ms
int defaultSpeed = 120; // max = 256
int turnSpeed1 = 200; // The speed of the forward going wheel
int turnSpeed2 = 200; // backward

bool backLeft = true;

void setup(){
  // zeg welke 'pin' invoer en uitvoer is
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(controlPin12, OUTPUT);
  pinMode(controlPin22, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);  

  setMotors(1, 0, 1, 0);
}


void loop(){
  println(getInfSensor(2) + " " + getInfSensor(3));
  // getAction();
  // if (getAction() == 1) {
  //   delay(turnTimeout);
  //   setMotors(1, turnSpeed1, 0, turnSpeed2);
  //   delay(turnTime);
  // }
  // else if (getAction() == 0) {
  //   delay(forwardTimeout);
  //   goForwards(defaultSpeed);
  //   delay(forwardTime);
  // }
  // else if (getAction() == -1) {
  //   delay(turnTimeout);
  //   setMotors(0, turnSpeed2, 1, turnSpeed1);
  //   delay(turnTime);
  // }
  // else if (getAction() == 2) {
  //   delay(50);
  //   // goBackwards(defaultSpeed);
  //   // delay(backTime);
  //   if (backLeft) {
  //     setMotors(1, defaultSpeed, 0, defaultSpeed+100);
  //     delay(backTime);
  //   } else {
  //     setMotors(0, defaultSpeed+100, 1, defaultSpeed);
  //     delay(backTime);
  //   }

  //   backLeft = !backLeft;
  // }
  // stop();

}


// CAR CONTROL FUNCTIONS ----------------------

int retBlackSensor() {
  // NEVER TESTED, PROCEED WITH CAUTION
  // sensor numbers from left to right: -2 1 0 1 2
  // this function returns which sensor is detecting black
  // if two or more sensors are detecting black, it outputs the furtherest one (i.e. if both 0 and 1 are on black, it outputs 1)

  int blackSensor = 0;

  for (int sensorNum=-2; sensorNum<3; sensorNum++) {

    pinMode(IFpins[sensorNum], OUTPUT);
    pinMode(IFpins[sensorNum], OUTPUT);
    digitalWrite(IFpins[sensorNum], HIGH);
    digitalWrite(IFpins[sensorNum], HIGH);
    delay(1);
    pinMode(IFpins[sensorNum], INPUT);
    pinMode(IFpins[sensorNum], INPUT);
    delay(IFpins[sensorNum]);


    // sensorNum+2 because the leftmost sensor (sensorNum = 2) has pin index of 0 (in the IFpins list declare at the very top)
    sensorIsBlack = digitalRead(IFpins[sensorNum+2]) == HIGH;

    // if the sensor touches the black line, then update the return value (to the furtherest one)
    if(sensorIsBlack) {
      
      if (abs(sensorIsBlack)) > abs(blackSensor)) {
        blackSensor = sensorNum
      }
    }
  }

  return blackSensor;
}

int getAction() {
  // turn right is 1 
  // forward is 0
  // turn left is -1 
  // backwards is 2
    pinMode(rightIFpin, OUTPUT);
    pinMode(leftIFpin, OUTPUT);
    digitalWrite(rightIFpin, HIGH);
    digitalWrite(leftIFpin, HIGH);
    delay(1);
    pinMode(rightIFpin, INPUT);
    pinMode(leftIFpin, INPUT);
    delay(thresholdR);

    right = digitalRead(rightIFpin) == HIGH;
    left = digitalRead(leftIFpin) == HIGH;


  // Serial.print(left);
  // Serial.print("\t");
  // Serial.println(right);
  if (right && left) {
    return 2;
  }
  if (right) {
    return 1;
  }
  if (left) {
    return -1;
  }
  return 0;
}

void goAroundObstacle() {
  dist = getDistance();
  Serial.println(dist);
  if(dist > 30)
  {
    setMotors(1, defaultSpeed, 1, defaultSpeed);
  }
  else
  {
    setMotors(1,0,1,0);
  }
  delay(50);
}

int getInfSensor(int infpin) {
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

void goForwards(int speed) {
  setMotors(1, speed, 1, speed);
} 

void goBackwards(int speed) {
  setMotors(0, speed, 0, speed);
}

void stop() {
  setMotors(1, 0, 1, 0);
}

void setMotors(int r1, int s1, int r2, int s2) {
  // forward -> r1 = 1
  // r1 = 1 -r1;
  // r2 = 1 -r2;

  if(r1 == 1) {
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);

  } 
  else {
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
  } 

  // foward -> r2 = 1
  if(r2 == 1) {
    digitalWrite(controlPin12, LOW);
    digitalWrite(controlPin22, HIGH);
  } 
  else {
    digitalWrite(controlPin12, HIGH);
    digitalWrite(controlPin22, LOW);
  }

  // sets the speed
  // accomodate the left motor
  analogWrite(enablePin,  int(s1*1.1));
  analogWrite(enablePin2, s2);  
}

// ULTRASONIC FUNCTIONS -------------------
long getDistance() {
  //  pinMode(trigPin, OUTPUT);
   digitalWrite(trigPin, LOW);
   delayMicroseconds(5);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
  //  pinMode(echoPin, INPUT);

  // time out is optional and in ms, duration is in ms
   duration = pulseIn(echoPin, HIGH);

  //  Serial.println(duration);

   // returns in cm
  dist = duration * 0.017;
  //  return float (duration/2000) * 340;
  return dist;
}
