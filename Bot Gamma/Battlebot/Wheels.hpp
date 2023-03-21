// Libraries
#include "Bluetooth.hpp"
#include "Gripper.hpp"
#include "EchoLocator.hpp"

// Motor
const int rightWheelSensor = 9;
const int leftWheelFwd = 11;
const int leftWheelBwd = 6;
const int rightWheelBwd = 5;
const int rightWheelFwd = 3;
const int rotationSpeed = 160; // Speed at which to rotate
const int driveSpeed = 150; // Speed at which to drive
int actualSpeed = 150; // The currently set speed to the motors

// PID constants
const float KP = 0.225;
const float KD = 2.25;

int lastError = 0;
int wheelSensorCounter = 0;
int lastWheelSensor = 0;


//rotatedLeftLast
bool rotatedLeftLast;

void readRightWheelSensor()
{
  if (analogRead(rightWheelSensor) != lastWheelSensor)
  {
    wheelSensorCounter++;
    lastWheelSensor = analogRead(rightWheelSensor);
  }
}

// Forward
void driveLeftWheel(int speed)
{
  analogWrite(leftWheelBwd, 0);
  analogWrite(leftWheelFwd, speed);
}

void driveRightWheel(int speed)
{
  analogWrite(rightWheelBwd, 0);
  analogWrite(rightWheelFwd, speed);
}

void driveFwd(bool doLights)
{
  if (doLights)
  {
    neoBack(50, 0, 0);
    neoFront(50, 50, 50);
  }
  driveLeftWheel(actualSpeed);
  driveRightWheel(actualSpeed);
}

// Backwards
void reverseLeftWheel()
{
  analogWrite(leftWheelFwd, 0);
  analogWrite(leftWheelBwd, 180);
}

void reverseRightWheel()
{
  analogWrite(rightWheelFwd, 0);
  analogWrite(rightWheelBwd, 180);
}

void driveBwd(bool doLights)
{
  if (doLights)
  {
    neoBack(150, 0, 0);
    neoFront(50, 50, 50);
  }
  reverseLeftWheel();
  reverseRightWheel();
}
void slowLeftWheel()
{
  analogWrite(leftWheelFwd, actualSpeed / 3);
}

void slowRightWheel()
{
  analogWrite(rightWheelFwd, actualSpeed / 3);
}

// Breaking
void breakLeftWheel()
{
  analogWrite(leftWheelFwd, 0);
  analogWrite(leftWheelBwd, 0);
}

void breakRightWheel()
{
  analogWrite(rightWheelFwd, 0);
  analogWrite(rightWheelBwd, 0);
}

void driveBreak(bool doLights)
{
  if (doLights)
  {
    neoBack(150, 0, 0);
    neoFront(150, 150, 150);
  }
  breakLeftWheel();
  breakRightWheel();
}

// Rotation
void rotateRight(bool doLights)
{
  if (doLights)
  {
    neoRight(0, 0, 0);
    neoLeft(150, 50, 0);
  }
  driveBreak(false);
  driveLeftWheel(actualSpeed);
  reverseRightWheel();
}

void rotateLeft(bool doLights)
{
  if (doLights)
  {
    neoRight(150, 50, 0);
    neoLeft(0, 0, 0);
  }
  driveBreak(false);
  driveRightWheel(actualSpeed);
  reverseLeftWheel();
}

void rotateRight2()
{
  driveBreak(false);
  driveLeftWheel(255);
  driveRightWheel(0);
}

void rotateLeft2()
{
  driveBreak(false);
  driveRightWheel(255);
  driveLeftWheel(0);
}

void driveAdvanced(int error)
{
  // Calculating turns
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  // Calculating motor speeds
  int m1Speed = 255 + motorSpeed;
  int m2Speed = 255 - motorSpeed;

  // Min and max speeds 
  m1Speed = min(max(m1Speed, 0), 255);
  m2Speed = min(max(m2Speed, 0), 255);

  // starting
  analogWrite(leftWheelBwd, 0);
  analogWrite(rightWheelBwd, 0);
  analogWrite(leftWheelFwd, m1Speed);
  analogWrite(rightWheelFwd, m2Speed);
}

void setupWheels()
{    
  // Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);
  pinMode(rightWheelSensor, INPUT);
  driveBreak(false);
}