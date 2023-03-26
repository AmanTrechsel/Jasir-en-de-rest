// Libraries
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include "Melodies.h"

// Constants
const int CALIBRATION_DRIVE_DISTANCE = 20;
const int CALIBRATION_DRIVE_SPEED = 200;
const int START_DRIVE_DISTANCE = 20; // Distance to travel to exit the start square
const int KICK_DRIVE_SPEED = 255;
const int KICK_DRIVE_DELAY = 100;
const int INTERSECTION_CHECK_DRIVE_DISTANCE = 10;
const int BASE_DRIVE_SPEED = 200;
const int BASE_ROTATION_SPEED = 170; // Speed at which to rotate at default
const float RIGHT_WHEEL_CORRECTION_MULTIPLIER = 0.9; // Multiplier to the right wheel since the left wheel is weaker
const float WHEEL_ROTATION_TICK_DEGREES = 8.5; // Degrees the bot rotates per 'tick'
const int CALIBRATION_CORRECTION_VALUE = 100; // An error factor that is added/removed for the white/black thresholds
const int ROTATION_CORRECTION_DRIVE_DISTANCE = 31; // Distance to drive forward in order to compensate for a 90 degree turn
const int START_SIGNAL_DISTANCE = 20; // Distance it needs to see in order to start
const int START_SIGNAL_WAIT = 5000; // Time to wait once it has received its start signal (in milliseconds)
const int DRIVING_LIGHTS_TIME = 100; // Time in milliseconds between switching the lights while driving normally
// Range of sensors to check for left turns
const int LEFT_RANGE_MIN = 0;
const int LEFT_RANGE_MAX = 2;
// Range of sensors to check for right turns
const int RIGHT_RANGE_MIN = 5;
const int RIGHT_RANGE_MAX = 7;
// PID constants
const float PROPORTIONAL_GAIN = 0.225;
const float DERIVATIVE_GAIN = 2.25;
// Gripper
const int GRIPPER_CLOSED_ANGLE = 130;
const int GRIPPER_OPENED_ANGLE = 180;

// Pins
// UNUSED = 2;
const int PIN_RIGHT_WHEEL_FORWARD = 3;
// UNUSED = 4;
const int PIN_RIGHT_WHEEL_BACKWARD = 5;
const int PIN_LEFT_WHEEL_BACKWARD = 6;
const int PIN_NEO_PIXEL = 7;
// PIN_TONE = 8;
const int PIN_RIGHT_WHEEL_SENSOR = 9;
const int PIN_GRIPPER = 10;
const int PIN_LEFT_WHEEL_FORWARD = 11;
const int PIN_CLICKER_SPEAKER = 12;
const int PIN_CLICKER_MICROPHONE = 13;

// Sensor related
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;
int blackThreshold;
int whiteThreshold;

// Wheel related
int lastError;
int wheelSensorCounter;
int wheelSensorCounterLast;

// NEO PIXELS
Adafruit_NeoPixel neo(4, PIN_NEO_PIXEL, NEO_RGB + NEO_KHZ800);

// States
bool driving;
bool checkedBlack;
bool finished;

// Timers
unsigned long timerStartSignal;
unsigned long timerDrivingLights;

// Setup
void setup()
{
  // NEO setup
  neo.begin();
  neo.clear();
  pinMode(PIN_TONE, OUTPUT);
  // Wheels
  pinMode(PIN_RIGHT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_SENSOR, INPUT);

  // Gripper
  pinMode(PIN_GRIPPER, OUTPUT);

  // Clicker
  pinMode(PIN_CLICKER_MICROPHONE, OUTPUT);
  pinMode(PIN_CLICKER_SPEAKER, INPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);

  // Await start
  unsigned int waitCounter = 0;
  int waitColor = 0;
  int waitColorLast = 0;
  while (calculateDistance() > START_SIGNAL_DISTANCE)
  {
    waitColor = (sin(waitCounter/5.0)+1.1)*50;
    waitColorLast = waitColor/10;
    switch(waitCounter%4)
    {
      case 0:
        neoFrontLeft(waitColor,waitColor,waitColor);
        neoFrontRight(waitColorLast,waitColorLast,waitColorLast);
        break;
      case 1:
        neoFrontRight(waitColor,waitColor,waitColor);
        neoBackLeft(waitColorLast,waitColorLast,waitColorLast);
        break;
      case 2:
        neoBackLeft(waitColor,waitColor,waitColor);
        neoBackRight(waitColorLast,waitColorLast,waitColorLast);
        break;
      case 3:
        neoBackRight(waitColor,waitColor,waitColor);
        neoFrontLeft(waitColorLast,waitColorLast,waitColorLast);
        break;
    }
    delay(100);
    neo.clear();
    waitCounter++;
  }

  openGripper();

  // Start signal delay
  timerStartSignal = millis() + START_SIGNAL_WAIT;
  int startCounter = 0;
  while (timerStartSignal > millis())
  {
    int waitColor = (sin(startCounter/50.0)+1.1)*50;
    neoFull(waitColor,waitColor,waitColor);
    delay(1);
    startCounter++;
  }

  unsigned long timerPacmanMelody = 0;
  int pacmanNoteCounter = 0;
  while (pacmanNoteCounter < PACMAN_NOTES || millis() < timerPacmanMelody)
  {
    if (millis() >= timerPacmanMelody)
    {
      neo.clear();
      timerPacmanMelody = millis() + playPacman();
      pacmanNoteCounter++;
      if (pacmanNoteCounter == PACMAN_NOTES)
      {
        neoFull(100,80,10);
      }
      else if (pacmanNoteCounter >= PACMAN_SECOND_HALF_START)
      {
        switch(pacmanNoteCounter%4)
        {
          case 0:
            neoFrontLeft(70,50,0);
            break;
          case 1:
            neoFrontRight(70,50,0);
            break;
          case 2:
            neoBackLeft(70,50,0);
            break;
          case 3:
            neoBackRight(70,50,0);
            break;
        }
      }
      else
      {
        switch(pacmanNoteCounter%4)
        {
          case 0:
            neoFrontLeft(10,7,0);
            break;
          case 1:
            neoFrontRight(10,7,0);
            break;
          case 2:
            neoBackLeft(10,7,0);
            break;
          case 3:
            neoBackRight(10,7,0);
            break;
        }
      }
    }
  }
  noTone(PIN_TONE);

  // Calibration
  driveForward(CALIBRATION_DRIVE_SPEED);
  int wheelSensorCounterLastCheck = 0;
  while (wheelSensorCounter < CALIBRATION_DRIVE_DISTANCE)
  {
    qtr.calibrate();
    readLine();
    readWheels();
    if (wheelSensorCounter != wheelSensorCounterLastCheck)
    {
      neo.clear();
      neoFull(random(150),random(150),random(150));
      wheelSensorCounterLastCheck = wheelSensorCounter;
    }
  }
  closeGripper();
  rotateWheels(90, CALIBRATION_DRIVE_SPEED, 0);
  driveForward(CALIBRATION_DRIVE_SPEED);
  resetWheelCounters();
  while (wheelSensorCounter < START_DRIVE_DISTANCE)
  {
    readLine();
    readWheels();
    if (wheelSensorCounter != wheelSensorCounterLastCheck)
    {
      neo.clear();
      neoFull(random(150),random(150),random(150));
      wheelSensorCounterLastCheck = wheelSensorCounter;
    }
  }
  int thresholdCounterBlack = 0;
  int thresholdCounterWhite = 0;
  for (int i = 0; i < 8; i++)
  {
    thresholdCounterBlack += qtr.calibrationOn.maximum[i];
    thresholdCounterWhite += qtr.calibrationOn.minimum[i];
  }
  blackThreshold = thresholdCounterBlack / 8 - CALIBRATION_CORRECTION_VALUE;
  whiteThreshold = thresholdCounterWhite / 8 + CALIBRATION_CORRECTION_VALUE;
  neo.clear();
}

// Loop
void loop()
{
  tone(PIN_TONE, random(4947)+31);
  readLine();
  if (finished)
  {
    openGripper();
    breakWheels();
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, KICK_DRIVE_SPEED);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, KICK_DRIVE_SPEED);
    unsigned long timerMarioMelody = 0;
    while (true)
    {
      if (millis() >= timerMarioMelody)
      {
        neo.clear();
        neoFrontLeft(random(150),random(150),random(150));
        neoFrontRight(random(150),random(150),random(150));
        neoBackLeft(random(150),random(150),random(150));
        neoBackRight(random(150),random(150),random(150));
        timerMarioMelody = millis() + playMario();
      }
    }
  }
  else
  {
    if (!driving)
    {
      drivePID(BASE_DRIVE_SPEED);
      driving = true;
    }
    else
    {
      if (readBlackLine())
      {
        resetWheelCounters();
        while (wheelSensorCounter < INTERSECTION_CHECK_DRIVE_DISTANCE)
        {
          readLine();
          readWheels();
        }
        if (readBlackLine())
        {
          noTone(PIN_TONE);
          neoFull(128,128,128);
          breakWheels();
          openGripper();
          delay(400);
          driveBackward(BASE_DRIVE_SPEED);
          delay(400);
          breakWheels();
          delay(400);
          finished = true;
          breakWheels();
        }
        else
        {
          driveBackward(BASE_DRIVE_SPEED);
          resetWheelCounters();
          while (wheelSensorCounter < INTERSECTION_CHECK_DRIVE_DISTANCE)
          {
            readLine();
            readWheels();
          }
          rotateWheels(90, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
        }
      }
      else if (readWhiteLine())
      {
        rotateWheels(90, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
      }
      else if (readRightLine())
      {
        rotateWheels(90, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
      }
      else if (readLeftLine())
      {
        rotateWheels(-90, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
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

bool readWhiteLine()
{
  for (int i = 0; i < 8; i++)
  {
    if (sensors[i] > whiteThreshold)
    {
      return false;
    }
  }
  return true;
}

bool readLeftLine()
{
  for (int i = LEFT_RANGE_MIN; i < LEFT_RANGE_MAX; i++)
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
  for (int i = RIGHT_RANGE_MIN; i < RIGHT_RANGE_MAX; i++)
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
  if (analogRead(PIN_RIGHT_WHEEL_SENSOR) != wheelSensorCounterLast)
  {
    wheelSensorCounter++;
    wheelSensorCounterLast = analogRead(PIN_RIGHT_WHEEL_SENSOR);
  }
}

void resetWheelCounters()
{
  wheelSensorCounter = 0;
  wheelSensorCounterLast = 0;
}

void breakWheels()
{
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
}

void driveForward(unsigned short int speed)
{ 
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, KICK_DRIVE_SPEED);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, KICK_DRIVE_SPEED);
  delay(KICK_DRIVE_DELAY);
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
}

void driveBackward(unsigned short int speed)
{
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, KICK_DRIVE_SPEED * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, KICK_DRIVE_SPEED);
  delay(KICK_DRIVE_DELAY);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
}

void rotateWheels(int degrees, int speed, int compensation_distance)
{
  float degreesLeft = WHEEL_ROTATION_TICK_DEGREES * abs(degrees);
  resetWheelCounters();
  breakWheels();
  driveForward(speed);
  while (wheelSensorCounter < compensation_distance)
  {
    readWheels();
  }
  resetWheelCounters();
  breakWheels();
  neo.clear();
  if (degrees < 0)
  {
    neoLeft(0,100,30);
    neoRight(20,5,0);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, KICK_DRIVE_SPEED);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, KICK_DRIVE_SPEED * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
    delay(KICK_DRIVE_DELAY);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
    while (wheelSensorCounter < degreesLeft)
    {
      readWheels();
    }
  }
  else
  {
    neoLeft(20,5,0);
    neoRight(0,100,30);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, KICK_DRIVE_SPEED * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, KICK_DRIVE_SPEED);
    delay(KICK_DRIVE_DELAY);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
    while (wheelSensorCounter < degreesLeft)
    {
      readWheels();
    }
  }
  driving = false;
  breakWheels();
}

void drivePID(int speed)
{
  // Calculating turns
  int error = lineReadData - 3500;
  int motorSpeed = PROPORTIONAL_GAIN * error + DERIVATIVE_GAIN * (error - lastError);
  lastError = error;

  // Calculating motor speeds
  int m1Speed = speed + motorSpeed;
  int m2Speed = speed - motorSpeed;

  // Min and max speeds 
  m1Speed = min(max(m1Speed, 0), 255);
  m2Speed = min(max(m2Speed, 0), 255);

  // Actually drive
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, m1Speed);
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, m2Speed);

  // Do some lights
  if (timerDrivingLights <= millis())
  {
    neo.clear();
    neoFront(random(30),random(150),random(150));
    neoBack(random(150),random(30),random(70));
    timerDrivingLights = millis() + DRIVING_LIGHTS_TIME;
  }
}

// Gripper
void openGripper()
{
  analogWrite(PIN_GRIPPER, GRIPPER_OPENED_ANGLE);
}

void closeGripper()
{
  analogWrite(PIN_GRIPPER, GRIPPER_CLOSED_ANGLE);
}

// Clicker
int calculateDistance()
{
  // Echo Locator
  digitalWrite(PIN_CLICKER_MICROPHONE, LOW);
  delayMicroseconds(20);
  digitalWrite(PIN_CLICKER_MICROPHONE, HIGH);
  delayMicroseconds(100);
  digitalWrite(PIN_CLICKER_MICROPHONE, LOW);

  // Receive Distance
  long duration = pulseIn(PIN_CLICKER_SPEAKER, HIGH);
  return duration / 2 * 0.034;
}

// Neo Pixel
void neoBackLeft(int r, int g, int b)
{
  neo.setPixelColor(1, neo.Color(r, g, b));
  neo.show();
}

void neoBackRight(int r, int g, int b)
{
  neo.setPixelColor(0, neo.Color(r, g, b));
  neo.show();
}

void neoFrontLeft(int r, int g, int b)
{
  neo.setPixelColor(2, neo.Color(r, g, b));
  neo.show();
}

void neoFrontRight(int r, int g, int b)
{
  neo.setPixelColor(3, neo.Color(r, g, b));
  neo.show();
}

void neoLeft(int r, int g, int b)
{
  neoBackLeft(r,g,b);
  neoFrontLeft(r,g,b);
}

void neoRight(int r, int g, int b)
{
  neoBackRight(r,g,b);
  neoFrontRight(r,g,b);
}

void neoBack(int r, int g, int b)
{
  neoBackLeft(r,g,b);
  neoBackRight(r,g,b);
}

void neoFront(int r, int g, int b)
{
  neoFrontLeft(r,g,b);
  neoFrontRight(r,g,b);
}

void neoFull(int r, int g, int b)
{
  neoFrontLeft(r,g,b);
  neoFrontRight(r,g,b);
  neoBackLeft(r,g,b);
  neoBackRight(r,g,b);
}