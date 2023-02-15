//*****LIBRARIES*****//

#include <Servo.h>

//*******************//

//*****CONSTANSTS*****//

const int gripperPin = 4; // the pin that is connected to the gripper
const int scannerPin = 12; // the pin that is connected tot the scanner servo
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotorA = 2; // rotation sensor for motor A (the left motor)
const int sensorMotorB = 3; // rotation sensor for motor B (the right motor)
const int numberOfHoles = 20; // number of holes in the speed sensors
const int nintyDegreeTurn = 400; // the amount of time the wheels need to work for a 90 degree turn
const int oneEightyDegreeTurn = 700; // the amount of time the wheels need to work for a 180 degree turn

//*******************//

//*****VARIABLES*****//

long duration; // the time it takes for the echo to be detected
int distance; // the distance between the sensor and the object
int pos = 0; 
int counter1 = 0;
int counter2= 0;

//*******************//

Servo gripper ;
Servo scanner ;


void setup() {
  pinMode(trigPin, OUTPUT); // sets the trigger pin as an output
  pinMode(echoPin, INPUT); // sets the echo pin as an input
  Serial.begin(9600); // starts the serial communication
  gripper.attach(gripperPin); // attaches the gripper servo to pin 4
  scanner.attach(scannerPin);
  
  pinMode(motorA1, OUTPUT); // sets the motorA1 pin as output
  pinMode(motorA2, OUTPUT); // sets the motorA2 pin as output 
  pinMode(motorB1, OUTPUT); // sets the motorB1 pin as output
  pinMode(motorB2, OUTPUT); // sets the motorB2 pin as output
  //pinMode(sensorMotorA, INPUT);
  //pinMode(sensorMotorB, INPUT);
  //attachInterrupt(digitalPinToInterrupt(sensorMotorA), counter1, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(sensorMotorB), counter2, CHANGE);
  
}

void loop () {
 
   drive();
  
}

void echoSensor() {
  digitalWrite(trigPin, LOW);
  delay(2);
  
  digitalWrite(trigPin, HIGH);
  delay(500);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("distance: ");
  Serial.println(distance);
}

void drive() {
  frontScan();
  echoSensor();
  forward();
  if (distance <= 15) {
    brake();
    delay(500);
    leftScan();
    delay(500);
    echoSensor(); 
    delay(1000);  
    if (distance <= 15) {
      rightScan(); 
      delay(500); 
      echoSensor();
      delay(1000);
      if (distance <= 15) {
        right();
        delay(nintyDegreeTurn);
        brake();
      }
      else {
        right();
        delay(nintyDegreeTurn);
        brake();
        delay(500);
       } 
     }
     else {
       left();
       delay(nintyDegreeTurn);
       brake();
       delay(500);
    } 
  }

}

void forward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void backward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void left() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void right() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void brake() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftScan() {
  scanner.write(180);
}

void rightScan() {
  scanner.write(0);
}

void frontScan() {
  scanner.write(100);
}

/* void counter1() {
  counter1++;
}

void counter2() {
  counter2++;
} */
