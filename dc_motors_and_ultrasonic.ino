// VARIABLES DEFINITION ----------------------------
// 2, 3, 9 is the left motor
const int controlPin1 = 2; // aangesloten aan pin 7 van de H-bridge
const int controlPin2 = 3; // aangesloten aan pin 2 van de H-bridge
const int enablePin = 9;   // aangesloten aan pin 1 van de H-bridge

// 6, 7, 10 is the right motor
const int controlPin12 = 6; // aangesloten aan pin 15 van de H-bridge
const int controlPin22 = 7; // aangesloten aan pin 10 van de H-bridge
const int enablePin2 = 10;   // aangesloten aan pin 9 van de H-bridge

const int trigPin =  4;
const int echoPin =  5; 
long duration;
float dist;

// other default values
int timeout = 50; // in ms
int defaultSpeed = 200;

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

  setMotors(1,0,1,0);
}


void loop(){
  dist=getDistance();
  Serial.println(dist);
  // if(dist > 30)
  // {
  //   setMotors(1, defaultSpeed, 1, defaultSpeed);
  // }
  // else
  // {
  //   // setMotors(1,0,1,0);
  //   goAroundObstacle();
  // }
  // delay(50);

  setMotors(0, 0, 0, defaultSpeed);
}


// CAR CONTROL FUNCTIONS ----------------------

// void goAroundObstacle() {
//   while() {

//   }
// }

void goFowards(int speed) {
  setMotors(1, speed, 1, speed);
} 

void goBackwards (int speed) {
  setMotors(0, speed, 0, speed);
}

void stop() {
  setMotors(1, 0, 1, 0);
}

void setMotors(int r1, int s1, int r2, int s2) {
  // forward -> r1 = 1
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
  analogWrite(enablePin,  s1);
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
