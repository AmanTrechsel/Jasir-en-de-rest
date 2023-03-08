
const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotor1 = 3; // rotation sensor for motor B (the right motor)
const int sensorMotor2 = 2; // rotation sensor for motor A (the left motor)
const int scanner = 12; // servo under the echoSensor
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
int counter1 = 0;
int counter2 = 0;
int pos = 0;
long duration; // the time it takes for the echo to be detected

int distance;
const int minSafeDistance = 15;



void setup() {
  pinMode(sensorMotor1, INPUT);
  pinMode(sensorMotor2, INPUT);
  pinMode(trigPin, OUTPUT); // sets the trigger pin as an output
  pinMode(echoPin, INPUT); // sets the echo pin as an input
  pinMode(scanner, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorMotor1), count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensorMotor2), count2, CHANGE);

}

void loop() {
  
  leftScan();
  delay(500);
  echoSensor();
  if (distance >= minSafeDistance)
  {
    turnLeft();
  }
  else 
  {
    rightScan();
    delay(500);
    echoSensor();
    if (distance >= minSafeDistance)
    {
      turnRight();
    }
    else
    {
      frontScan();
      delay(500);
      echoSensor();
      if (distance <= minSafeDistance)
      {
        turnAround();
      }
    }
  }
  
  /*boolean scanLeft = false;
  boolean scanRight = false;
  boolean scanFront = false;

  leftScan();
  delay(500);
  if (scanLeft = true)
  {
    echoSensorLeft();
    scanLeft = false;
  }
  if (scanLeft = false)
   {
     rightScan();
     delay(500);
     if (scanRight = true)
     {
       echoSensorRight();
       scanRight = false;
     }
     else
     {
       frontScan();
       delay(500);
       if (scanFront = true)
       {
         echoSensorFront();
         scanFront = false;
       }
     }
   }
  Serial.print("Left: ");
  Serial.println(distanceLeft);
  Serial.print("right: ");
  Serial.println(distanceRight);
  Serial.print("front: ");
  Serial.println(distanceFront);

  if (distanceLeft >= minSafeDistance)
  {
    turnLeft();
  }
  else if (distanceRight >= minSafeDistance)
  {
    turnRight();
  }
  else if (distanceRight < minSafeDistance && distanceLeft < minSafeDistance && distanceFront < minSafeDistance)
  {
    turnAround();
  }
  else if (distanceFront >= minSafeDistance)
  {
    squareUp();
  }*/
  
}

void count1() {
  counter1++;
}

void count2() {
  counter2++;
}

void squareUp () {
  forward();
  if (counter1 >= 20) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, HIGH);
  }
  if (counter2 >= 20) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, HIGH);
  }
}

void turnLeft () {
  left();
  if (counter2 >= 14) {
    leftBrake();
  }
  if (counter1 >= 14) {
    rightBrake();
  }
}

void turnRight () {
  right();
  if (counter2 >= 14) {
    leftBrake();
  }
  if (counter1 >= 14) {
    rightBrake();
  }
}

void turnAround () {
  left();
  if (counter2 >= 32) {
    leftBrake();
    rightBrake();
  }
}

void rightBrake () {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftBrake () {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

void leftScan () {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(2550);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
}

void rightScan () {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(550);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
}

void frontScan () {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(1550);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
}


void echoSensor () {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
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

void brake() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
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
