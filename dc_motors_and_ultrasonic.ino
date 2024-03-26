/*
 Robotauto RLO
 Modified 4 februari 2019
 by John Val
 
 This example code is part of the public domain 
 */

const int controlPin1 = 2; // aangesloten aan pin 7 van de H-bridge
const int controlPin2 = 3; // aangesloten aan pin 2 van de H-bridge
const int enablePin = 9;   // aangesloten aan pin 1 van de H-bridge

const int controlPin12 = 6; // aangesloten aan pin 15 van de H-bridge
const int controlPin22 = 7; // aangesloten aan pin 10 van de H-bridge
const int enablePin2 = 10;   // aangesloten aan pin 9 van de H-bridge

const int trigPin =  4; // Trigger Pin of Ultrasonic Sensor
const int echoPin =  5; // Echo Pin of Ultrasonic Sensor
const int lightPin = 13; // Echo Pin of Ultrasonic Sensor
long timeout=200;
long duration, cm;
/**
 * setup is de functie die als eerste wordt gestart
 * In deze functie worden de benodigde aansluitingen
 * in de juiste toestand gezet
 */
void setup(){
  // zeg welke 'pin' invoer en uitvoer is
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(controlPin12, OUTPUT);
  pinMode(controlPin22, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);  
  // om te starten zet de motoren uit
  setMotors(1,0,1,0);
}

/**
 * loop is de functie die na setup wordt aangeroepen.
 * Deze blijft doorgaan totdat de stroom van de Arduino
 * wordt afgehaald.
 * Ongeveer iedere timeout ms wordt een afstand meting gedaan.
 * Als de afstand kleiner is dan 30 cm stopt de auto en gaat 
 * een lichtje branden
 */
void loop(){
  cm=getDistance();
  Serial.println(cm);
  if(cm > 30)
  {
    setMotors(0,255,0,255);
    digitalWrite(lightPin, LOW);
  }
  else
  {
    setMotors(1,0,1,0);
    digitalWrite(lightPin, HIGH);
  }
  delay(timeout);
}

/**
 * setMotors zet de richting en de snelheid van
 * de motoren.
 * r1 : richting eerste motor ( 1 of anders)
 * s1 : snelheid van de eerste motor (0 t/m 255)
 * r2 : richting tweede motor ( 1 of anders)
 * s2 : snelheid van de tweede motor (0 t/m 255)
 */
void setMotors(int r1, int s1, int r2, int s2)
{
  if(r1 == 1) {
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
  } 
  else {
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
  }  
  if(r2 == 1) {
    digitalWrite(controlPin12, HIGH);
    digitalWrite(controlPin22, LOW);
  } 
  else {
    digitalWrite(controlPin12, LOW);
    digitalWrite(controlPin22, HIGH);
  }
  analogWrite(enablePin,  s1);
  analogWrite(enablePin2, s2);  
}

/**
 *  getDistance doet een geluids echo meting 
 *  en bepaald daarmee de afstand tot het dichtsbijzijnde
 *  object
 */
long getDistance()
{
   pinMode(trigPin, OUTPUT);
   digitalWrite(trigPin, LOW);
   delayMicroseconds(5);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH , timeout*1000 );
   return (duration/2) / 29.1;
}
