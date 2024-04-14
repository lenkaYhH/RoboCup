int pins[5] = {13,12,11,8,1}; //left to right pins for ir sensor

// 2, 3, 9 is the left motor
const int controlPin1 = 2; // aangesloten aan pin 7 van de H-bridge
const int controlPin2 = 3; // aangesloten aan pin 2 van de H-bridge
const int enablePin = 9;   // aangesloten aan pin 1 van de H-bridge

// 6, 7, 10 is the right motor
const int controlPin12 = 6; // aangesloten aan pin 15 van de H-bridge
const int controlPin22 = 7; // aangesloten aan pin 10 van de H-bridge
const int enablePin2 = 10;   // aangesloten aan pin 9 van de H-bridge


float ki=0;// coefficient for integral of error
float kd=0;// coefficient for derivative of error
float k=1;// coefficient for error

int forwardSpeed = 120; // forward speed
int maxTurn = 120; // max forward speed
int minTurn = -20; // max backward speed


bool sensors[5] = {false,false,false,false,false};
int sensorPos = 0;
const int threshold = 6;

// data required to calculate de/dt
unsigned long prevTime = 0;
int prevPos=0;

// data required to do integral of error
int integralList[128]; // The value of prev 128 error
int integralTime[128]; // The duration of the prev 128 error
int integralIndex=0; // point at the current overwriting value in list
long integralSum = 0;// won't overflow
int totalTime = 0;

int turnFactor;

// current activity
// 0:nothing, 1:going forward, 2:going backward, 3:turning left, 4:turning right
int current = 0;

void setMotors(int s1, int s2) {
  // move motors at s1, s2
  if(s1 > 0) {
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);

  } 
  else {
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
  } 

  // foward -> r2 = 1
  if(s2 > 0) {
    digitalWrite(controlPin12, LOW);
    digitalWrite(controlPin22, HIGH);
  } 
  else {
    digitalWrite(controlPin12, HIGH);
    digitalWrite(controlPin22, LOW);
  }

  analogWrite(enablePin,  abs(int(s1*1.1)));
  analogWrite(enablePin2, abs(s2));  
}


void readSensor(){
  // read data from sensor and store result in sensorPos
  // note: sensors should not be referenced in outside scope

  // prepare sensor
  for(int p:pins){
    pinMode(p, OUTPUT);
    digitalWrite(p,HIGH);
  }
  delay(1);
  for(int p:pins){
    pinMode(p, INPUT);
  }
  delay(threshold);

  // check if the value changed
  bool changed = false;
  int sv;
  for(int i=0;i<5;i++){
    sv = digitalRead(pins[i])==HIGH;//IF IT'S BLACK
    if(sensors[i]!=sv){
      changed=true;
      sensors[i]=sv;
    }
  }

  if(changed){// if changed, get new pos
    int blacknum = 0;
    sensorPos=0;
    for(int i=0;i<5;i++){
      if(sensors[i]){
        blacknum+=1;
        sensorPos += (i-2);
      }
    }
    // adjust for number of value added, make sure result is an integer;
    sensorPos = 60/blacknum*sensorPos;
  }
}

void calTurn(){
  // calculate integral and derivative of error to get the turn factor
  // integral
  unsigned long curTime = millis();
  integralSum -= integralList[integralIndex]*integralTime[integralIndex];
  totalTime -= integralTime[integralIndex];
  integralList[integralIndex] = sensorPos;
  integralTime[integralIndex] = (int)(curTime-prevTime);
  integralSum += sensorPos*(curTime-prevTime);
  totalTime += (int)(curTime-prevTime);
  integralIndex++;
  if (integralIndex==128)integralIndex=0;
  // turn factor
  turnFactor = (int)(integralSum*ki/totalTime + (sensorPos-prevPos)*kd/max(1,(curTime-prevTime)) + k*sensorPos);
  
}

void controller(){
  calTurn();
  setMotors(max(maxTurn,min(minTurn, forwardSpeed-turnFactor)), max(maxTurn,min(minTurn, forwardSpeed+turnFactor)));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // init integralList and integralTime
  for(int i=0;i<128;i++){
    integralList[i] = 0;
    integralTime[i] = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensor();
  controller();
}
