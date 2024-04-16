// Constants
// Please use define instead of creating variable

int pins[5] = {52,50,48,46,44}; //left to right pins for ir sensor

#define threshold 20

#define k 35.0 // coefficient for error, type is float
#define kd 10.0 // coefficient for derivative of error
#define ki 35.0 // coefficient for integral of error

#define iDur 128 // length of array

// 2, 3, 9 is the left motor
#define controlPin1 2 // aangesloten aan pin 7 van de H-bridge
#define controlPin2 3 // aangesloten aan pin 2 van de H-bridge
#define enablePin 4   // aangesloten aan pin 1 van de H-bridge

// 6, 7, 10 is the right motor
#define controlPin12 5 // aangesloten aan pin 15 van de H-bridge
#define controlPin22 6 // aangesloten aan pin 10 van de H-bridge
#define enablePin2 7  // aangesloten aan pin 9 van de H-bridge

// Ultrasonic
#define trigPin1 4
#define echoPin1 5

#define forwardSpeed 60 // forward speed
#define maxTurn 180 // max forward speed
#define minTurn -220 // max backward speed

//#define reduceTurn ((int) ((turnFactor>0)? turnFactor*0.2 : -turnFactor*0.2)) // when turning speed is reduced by this*turnfactor

bool sensors[5] = {false,false,false,false,false};
int sensorPos = 0;

// data required to calculate de/dt
unsigned long prevTime = 0;
int prevPos=0;

// data required to do integral of error
int integralList[iDur]; // The value of prev 128 error
int integralTime[iDur]; // The duration of the prev 128 error
int integralIndex=0; // point at the current overwriting value in list
long integralSum = 0;// won't overflow
int totalTime = 0;

int turnFactor; // >0:turning right, <0:turning left

// current activity
// 0:nothing, 1:going forward, 2:going backward, 3:turning left, 4:turning right
int current = 0;

//ultrasonic
float dist1; // distance in cm

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

// ULTRASONIC FUNCTIONS -------------------
void getDistance() {
   digitalWrite(trigPin1, LOW);
   delayMicroseconds(5);
   digitalWrite(trigPin1, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin1, LOW);

  // time out is optional and in ms, duration is in ms
   dist1 = pulseIn(echoPin1, HIGH) * 0.017;
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
    sensorPos = 12/blacknum*sensorPos;
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
  if (integralIndex==iDur)integralIndex=0;
  // turn factor
  turnFactor = (int)(integralSum*ki/totalTime + (sensorPos-prevPos)*kd/max(1,(curTime-prevTime)) + k*sensorPos);
  
}

void controller() {
  calTurn();
  Serial.println(turnFactor);
  Serial.print("\t");
  for (bool i:sensors){
    Serial.print(i);
    Serial.print(", ");
  }
  if (sensorPos!=prevPos){
    setMotors(0,0);
    //delay(50);
  }
  setMotors(min(maxTurn,max(minTurn, forwardSpeed+turnFactor)), min(maxTurn,max(minTurn, forwardSpeed-turnFactor)));
  prevTime = millis();
  prevPos = sensorPos;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  // do not do this, this mess up pin 0 and 1

  // set mode for motor pins
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(controlPin12, OUTPUT);
  pinMode(controlPin22, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // set mode for ultrasonic
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

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
