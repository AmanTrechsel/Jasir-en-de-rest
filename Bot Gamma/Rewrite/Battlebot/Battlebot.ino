// Libraries
#include <QTRSensors.h>

// Constants
const int CALIBRATION_DISTANCE = 20;
const int CALIBRATION_SPEED = 200;
const int START_DISTANCE = 20; // Distance to travel to exit the start square
const int KICK_POWER = 255;
const int KICK_TIMEOUT = 100;
const int CROSS_DISTANCE = 10;
const int BASE_SPEED = 200;
const int BASE_ROTATE = 150; // Speed at which to rotate at default
const int TURN_RIGHT_THRESHOLD = 5500;
const int TURN_LEFT_THRESHOLD = 1500;
const int WEAKNESS_FACTOR = -15; // Speed to add to the right wheel since it is weaker
const double DEGREES_FACTOR = 0.15;
const int BLACK_FACTOR = 150; // An error factor that is removed for the sensors

// Pins
const int PIN_RIGHT_WHEEL_FORWARD = 3;
const int PIN_RIGHT_WHEEL_BACKWARD = 5;
const int PIN_LEFT_WHEEL_BACKWARD = 6;
const int PIN_RIGHT_WHEEL_SENSOR = 9;
const int PIN_LEFT_WHEEL_FORWARD = 11;
const int PIN_LEFT_WHEEL_SENSOR = 10;

// Sensor related
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;
int blackThreshold;

// Wheel related
int wheelSensorCounter;
int leftWheelSensorCounter;
int rightWheelSensorCounter;

int leftWheelSensorCounterLast;
int rightWheelSensorCounterLast;

// States
bool driving;
bool checkedBlack;
bool finished;

// Setup
void setup()
{
  // Wheels
  pinMode(PIN_RIGHT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_SENSOR, INPUT);
  pinMode(PIN_LEFT_WHEEL_SENSOR, INPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);

  // Calibration
  driveForward(CALIBRATION_SPEED);
  while (wheelSensorCounter < CALIBRATION_DISTANCE)
  {
    qtr.calibrate();
    readLine();
    readWheels();
  }
  rotateWheels(-90, CALIBRATION_SPEED);
  driveForward(CALIBRATION_SPEED);
  resetWheelCounters();
  while (wheelSensorCounter < START_DISTANCE)
  {
    readLine();
    readWheels();
  }
  int threshold = 0;
  for (int i = 0; i < 8; i++)
  {
    threshold = qtr.calibrationOn.maximum[i];
  }
  blackThreshold = 900;//threshold / 8 - BLACK_FACTOR;
}

// Loop
void loop()
{
  if (finished)
  {

  }
  else
  {
    if (!driving)
    {
      driveForward(BASE_SPEED);
      driving = true;
    }
    else
    {
      if (readBlackLine())
      {
        resetWheelCounters();
        while (wheelSensorCounter < CROSS_DISTANCE)
        {
          readLine();
          readWheels();
        }
        if (readBlackLine())
        {
          finished = true;
          breakWheels();
        }
        else
        {
          driveBackward(BASE_SPEED);
          resetWheelCounters();
          while (wheelSensorCounter < CROSS_DISTANCE)
          {
            readLine();
            readWheels();
          }
          rotateWheels(90, BASE_ROTATE);
        }
      }
      else if (readRightLine())
      {
        rotateWheels(90, BASE_ROTATE);
      }
      else if (readLeftLine())
      {
        rotateWheels(-90, BASE_ROTATE);
      }
    }
  }
}

// Methods
bool readBlackLine()
{
  for (int i = 0; i < 8; i++)
  {
    if (sensors[i] < blackThreshold)
    {
      return false;
    }
  }
  return true;
}

bool readLeftLine()
{
  for (int i = 0; i < 3; i++)
  {
    if (sensors[i] >= blackThreshold)
    {
      return true;
    }
  }
  return false;
}

bool readRightLine()
{
  for (int i = 5; i < 8; i++)
  {
    if (sensors[i] >= blackThreshold)
    {
      return true;
    }
  }
  return false;
}

void readLine()
{
  lineReadData = qtr.readLineBlack(sensors);
}

void readWheels()
{
  bool hasIncremented = false;
  if (analogRead(PIN_RIGHT_WHEEL_SENSOR) != rightWheelSensorCounterLast)
  {
    rightWheelSensorCounter++;
    rightWheelSensorCounterLast = analogRead(PIN_RIGHT_WHEEL_SENSOR);
    hasIncremented = true;
  }
  if (analogRead(PIN_LEFT_WHEEL_SENSOR) != leftWheelSensorCounterLast)
  {
    leftWheelSensorCounter++;
    leftWheelSensorCounterLast = analogRead(PIN_LEFT_WHEEL_SENSOR);
    hasIncremented = true;
  }
  if (hasIncremented)
  {
    wheelSensorCounter++;
  }
}

void resetWheelCounters()
{
  wheelSensorCounter = 0;
  leftWheelSensorCounter = 0;
  rightWheelSensorCounter = 0;
  
  leftWheelSensorCounterLast = 0;
  rightWheelSensorCounterLast = 0;
}

void breakWheels()
{
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
}

void driveForward(int speed)
{ 
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, KICK_POWER + WEAKNESS_FACTOR);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, KICK_POWER);
  delay(KICK_TIMEOUT);
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed + WEAKNESS_FACTOR);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
}

void driveBackward(int speed)
{
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, KICK_POWER + WEAKNESS_FACTOR);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, KICK_POWER);
  delay(KICK_TIMEOUT);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed + WEAKNESS_FACTOR);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
}

void rotateWheels(int degrees, int speed)
{
  int degreesLeft = abs(degrees) / DEGREES_FACTOR;
  resetWheelCounters();
  breakWheels();
  if (degrees < 0)
  {
    analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, KICK_POWER);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, KICK_POWER + WEAKNESS_FACTOR);
    delay(KICK_TIMEOUT);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed + WEAKNESS_FACTOR);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
    while (degreesLeft > rightWheelSensorCounter)
    {
      readWheels();
    }
  }
  else
  {
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, KICK_POWER + WEAKNESS_FACTOR);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, KICK_POWER);
    delay(KICK_TIMEOUT);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed + WEAKNESS_FACTOR);
    while (degreesLeft > leftWheelSensorCounter)
    {
      readWheels();
    }
  }
  driving = false;
  breakWheels();
}