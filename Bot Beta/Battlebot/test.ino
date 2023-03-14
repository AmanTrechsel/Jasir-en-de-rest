
const int motorA1 = 10; // the first pin that is connected to motor A (the left motor)
const int motorA2 = 9; // the second pin that is connected to motor A (the left motor)
const int motorB1 = 6; // the first pin that is connected to motor B (the right motor)
const int motorB2 = 5; // the second pin that is connected to motor B (the right motor)
const int sensorMotor1 = 3; // rotation sensor for motor B (the right motor)
const int sensorMotor2 = 2; // rotation sensor for motor A (the left motor)
const int scanner = 12; // servo under the echoSensor
const int gripper = 4;
const int trigPin = 7; // the pin that is connected to the trigger of the ultra sonic sensor
const int echoPin = 8; // the pin that is connected to the echo of the ultra sonic sensor
int counter1 = 0;
int counter2 = 0;
int pos = 0;
int speed;
long duration; // the time it takes for the echo to be detected
boolean turnedAround = true;

int distance;
const int minSafeDistance = 20;



void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(sensorMotor1, INPUT);
  pinMode(sensorMotor2, INPUT);
  pinMode(trigPin, OUTPUT); // sets the trigger pin as an output
  pinMode(echoPin, INPUT); // sets the echo pin as an input
  pinMode(scanner, OUTPUT);
  pinMode(gripper, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorMotor1), count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensorMotor2), count2, CHANGE);

}

void loop() {
  /*if (turnedAround == false)
  {
    counter2 = 0;
    counter1 = 0;
    brake();
    leftScan();
    delay(500);
    echoSensor();
    if (distance <= minSafeDistance)
    {
      rightScan();
      delay(500);
      echoSensor();
      if(distance <= minSafeDistance)
      {
        frontScan();
        delay(500);
        echoSensor();
        if (distance <= minSafeDistance)
        {
          brake();
          turnAround();
          counter2 = 0;
          squareForward();
          turnedAround = true;
        }
        else
        {
          counter2 = 0;
          squareForward();
        }
      }
      else
      {
        brake();
        turnRight();
        counter2 = 0;
        squareForward();
      }
    }
    else
    {
      brake();
      turnLeft();
      counter2 = 0;
      squareForward();
    } 
  }
  else if (turnedAround == true)
  {
    counter2 = 0;
    counter1 = 0;
    brake();
    leftScan();
    delay(500);
    echoSensor();
    if (distance <= minSafeDistance)
    {
      frontScan();
      delay(500);
      echoSensor();
      if (distance <= minSafeDistance)
      {
        brake();
        turnAround();
        counter2 = 0;
        squareForward();
        turnedAround = true;
      }
      else
      {
        counter2 = 0;
        squareForward();
        turnedAround = false;
      }
    }
    else
    {
      brake();
      turnLeft();
      counter2 = 0;
      squareForward();
      turnedAround = false;
    }
  }*/
  counter2 = 0;
  counter1 = 0;
  brake();
  rightScan();
  delay(500);
  echoSensor();
  if (distance < minSafeDistance)
  {
    frontScan();
    delay(500);
    echoSensor();
    if (distance < minSafeDistance)
    {
      leftScan();
      delay(500);
      echoSensor();
      if (distance > minSafeDistance)
      {
        brake();
        turnLeft();
        counter2 = 0;
        squareForward();    
      }
      else
      {
        brake();
        turnAround();
      }
    }
    else
    {
      counter2 = 0;
      brake();
      squareForward();
    }
  }
  else
  {
    brake();
    turnRight();
    counter2 = 0;
    squareForward();
  }
}

void squareForward()
{
  while (counter2 < 52)
  {
    speed = 225;
    leftForward();
    rightForward();
  } 
}

void forwardCorrection()
{
  while (counter2 < 20)
  {
    speed = 225;
    leftForward();
    rightForward();
  }
}

void turnLeft()
{
  while (counter2 < 20)
  {
    speed = 175;
    leftBackward();
    rightForward();
   }
}

void turnRight()
{
  while (counter2 < 20)
  {
    speed = 175;
    leftForward();
    rightBackward();
  }
}

void turnAround()
{
  while (counter2 <= 30)
  {
    rightBackward();
  }
  while (counter2 > 31 && counter1 <= 30)
  {
    leftForward();
  }
}

void count1() {
  counter1++;
}

void count2() {
  counter2++;
}

void rightBrake () {
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftBrake () {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
}

void leftScan () 
{
  for (int i=0; i<10; i++)
  {
  digitalWrite(scanner, HIGH);
  delayMicroseconds(2550);
  digitalWrite(scanner, LOW);
  delayMicroseconds(18550);
  }
}

void rightScan () 
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(scanner, HIGH);
    delayMicroseconds(550);
    digitalWrite(scanner, LOW);
    delayMicroseconds(18550);
  }
}

void frontScan () 
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(scanner, HIGH);
    delayMicroseconds(1550);
    digitalWrite(scanner, LOW);
    delayMicroseconds(18550);
  }
}


void echoSensor () {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.println(distance);
  delay(10);
}

void brake() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, HIGH);
}

void leftForward()
{
  analogWrite(motorA1, 0);
  analogWrite(motorA2, speed);
}

void rightForward()
{
  analogWrite(motorB1, 0);
  analogWrite(motorB2, speed);
}

void leftBackward()
{
  analogWrite(motorA1, speed);
  analogWrite(motorA2, 0);
}

void rightBackward()
{
  analogWrite(motorB1, speed);
  analogWrite(motorB2, 0);
}
