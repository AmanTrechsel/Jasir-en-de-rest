// ################################################################################################################################
// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Bot Gamma *
//                                                   Main Program
// --------------------------------------------------------------------------------------------------------------------------------
//                                                 Jasir en de rest
//                                                   Version: 3.4
// --------------------------------------------------------------------------------------------------------------------------------
//                                                    Written by:
//                                                   Aman Trechsel
//                                                  Jasir Abdikarim
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
// ################################################################################################################################

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Libraries *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
#include <QTRSensors.h>                              // For sensors
#include <Adafruit_NeoPixel.h>                       // For lights
#include "Melodies.h"                                // For music

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Constants *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
const int MARIO_MELODY_AWAIT = 5000;                 // Time in milliseconds to wait before starting to play the mario tune
const int MAX_STRAIGHT_DIFFERENCE = 180;             // The maximum difference of power between the two wheels while driving with PID

// --------------------------------------------------------------------------------------------------------------------------------
//                                                     Starting
// --------------------------------------------------------------------------------------------------------------------------------
const int START_DRIVE_DISTANCE = 30;                 // Distance to travel to exit the start square
const int START_SIGNAL_DISTANCE = 30;                // Distance it needs to see in order to start
const int START_SIGNAL_WAIT = 2000;                  // Time to wait once it has received its start signal (in milliseconds)

// --------------------------------------------------------------------------------------------------------------------------------
//                                                    Calibration
// --------------------------------------------------------------------------------------------------------------------------------
const int CALIBRATION_DRIVE_DISTANCE = 25;           // Distance to drive when calibrating
const int CALIBRATION_DRIVE_SPEED = 180;             // Speed to drive when calibrating
const int CALIBRATION_CORRECTION_VALUE = 100;        // An error factor that is added/removed for the white/black thresholds

// --------------------------------------------------------------------------------------------------------------------------------
//                                                      Driving
// --------------------------------------------------------------------------------------------------------------------------------
const int BASE_DRIVE_SPEED = 180;                    // Speed at which to drive at default
const int BASE_ROTATION_SPEED = 200;                 // Speed at which to rotate at default
const float RIGHT_WHEEL_CORRECTION_MULTIPLIER = 0.9; // Multiplier to the right wheel since the left wheel is weaker
const float WHEEL_ROTATION_TICK_DEGREES = 22.5;      // Degrees the bot rotates per 'tick'
const int ROTATION_CORRECTION_DRIVE_DISTANCE = 1300; // Distance to drive forward in order to compensate for a 90 degree turn
const int DRIVING_LIGHTS_TIME = 100;                 // Time in milliseconds between switching the lights while driving normally
const int INTERSECTION_CHECK_DRIVE_DISTANCE = 18;    // Drive forward by this amount when detecting an intersection

                                                     // Range of sensors to check for left turns
const int LEFT_RANGE_MIN = 0;                        // Most left sensor
const int LEFT_RANGE_MAX = 1;                        // Most centered left sensor

                                                     // Range of sensors to check for right turns
const int RIGHT_RANGE_MIN = 6;                       // Most centered right sensor
const int RIGHT_RANGE_MAX = 7;                       // Most right sensor

                                                     // PID constants
const float PROPORTIONAL_GAIN = 0.225;               // Kp
const float DERIVATIVE_GAIN = 2.25;                  // Kd

// --------------------------------------------------------------------------------------------------------------------------------
//                                                      Gripper
// --------------------------------------------------------------------------------------------------------------------------------
const int GRIPPER_CLOSED_ANGLE = 120;                // Angle at which the gripper is closed
const int GRIPPER_OPENED_ANGLE = 180;                // Angle at which the gripper is opened

// --------------------------------------------------------------------------------------------------------------------------------
//                                                       Pins
// --------------------------------------------------------------------------------------------------------------------------------
const int PIN_RIGHT_WHEEL_FORWARD = 3;
const int PIN_RIGHT_WHEEL_BACKWARD = 5;
const int PIN_LEFT_WHEEL_BACKWARD = 6;
const int PIN_NEO_PIXEL = 7;
const int PIN_RIGHT_WHEEL_SENSOR = 9;
const int PIN_GRIPPER = 10;
const int PIN_LEFT_WHEEL_FORWARD = 11;
const int PIN_CLICKER_SPEAKER = 12;
const int PIN_CLICKER_MICROPHONE = 13;

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                   * Variables *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================

// --------------------------------------------------------------------------------------------------------------------------------
//                                                      Sensors
// --------------------------------------------------------------------------------------------------------------------------------
QTRSensors qtr;                                      // Handles sensor
uint16_t sensors[8];                                 // Array of sensor values
int lineReadData;                                    // A number between 0 and 7000 defining the 'position' of the sensor

// --------------------------------------------------------------------------------------------------------------------------------
//                                                      Driving
// --------------------------------------------------------------------------------------------------------------------------------
int lastError;                                       // Last drive error (for PID)
int wheelSensorCounter;                              // A counter for how many pulses are received by the wheel sensor
int wheelSensorCounterLast;                          // The last counter value for the wheel sensor

// --------------------------------------------------------------------------------------------------------------------------------
//                                                      Neo Pixel
// --------------------------------------------------------------------------------------------------------------------------------
Adafruit_NeoPixel neo(4, PIN_NEO_PIXEL, NEO_RGB + NEO_KHZ800);

// --------------------------------------------------------------------------------------------------------------------------------
//                                                       States
// --------------------------------------------------------------------------------------------------------------------------------
bool checkedBlack;                                   // Whether it has seen full black and checked it
                                                     //         (used for determining whether it is at the end of the course)
bool finished;                                       // Whether it has finished the course

// --------------------------------------------------------------------------------------------------------------------------------
//                                                       Timers
// --------------------------------------------------------------------------------------------------------------------------------
unsigned long timerStartSignal;                      // A timestamp for the start signal
unsigned long timerDrivingLights;                    // A timestamp for the driving lights

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                      * Setup *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
void setup()
{
  // NEO setup
  neo.begin();
  neo.clear();

  // Tone setup
  pinMode(PIN_TONE, OUTPUT);

  // Wheels setup
  pinMode(PIN_RIGHT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_WHEEL_BACKWARD, OUTPUT);
  pinMode(PIN_RIGHT_WHEEL_SENSOR, INPUT);

  // Gripper setup
  pinMode(PIN_GRIPPER, OUTPUT);

  // Clicker setup
  pinMode(PIN_CLICKER_MICROPHONE, OUTPUT);
  pinMode(PIN_CLICKER_SPEAKER, INPUT);

  // Line Sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  
  // Await start
  delay(1000);
  unsigned int waitCounter = 0;
  int waitColor = 0;
  int waitColorLast = 0;
  // Do something while awaiting start signal
  while (calculateDistance() > START_SIGNAL_DISTANCE)
  {
    waitColor = (sin(waitCounter/5.0)+1.1)*50;
    waitColorLast = waitColor/10;
    // Lights will move in a pattern
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

  // Open gripper for start
  openGripper();

  // Start signal delay
  timerStartSignal = millis() + START_SIGNAL_WAIT;
  int startCounter = 0;
  // Starting state
  while (timerStartSignal > millis())
  {
    int waitColor = (sin(startCounter/50.0)+1.1)*50;
    neoFull(waitColor,waitColor,waitColor);
    delay(1);
    startCounter++;
  }

  // Play pacman startup sound
  unsigned long timerPacmanMelody = 0;
  int pacmanNoteCounter = 0;
  while (pacmanNoteCounter < PACMAN_NOTES || millis() < timerPacmanMelody)
  {
    if (millis() >= timerPacmanMelody)
    {
      neo.clear();
      timerPacmanMelody = millis() + playPacman();
      pacmanNoteCounter++;
      // Light controls
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
  noTone(PIN_TONE); // Stop sound

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
  // Reached end of calibration
  closeGripper();
  // Rotate left
  rotateWheels(false, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
  // Drive a little to ensure that we exited the calibration zone
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
  neo.clear(); // Turn off the lights
}

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                      * Loop *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
void loop()
{
  readLine();
  if (finished)
  {
    // Reset values
    openGripper();
    breakWheels();
    unsigned long timerMarioMelody = millis() + MARIO_MELODY_AWAIT;
    songPosition = 0;
    while (true)
    {
      if (millis() >= timerMarioMelody)
      {
        // Disco lights
        neo.clear();
        neoFrontLeft(random(150),random(150),random(150));
        neoFrontRight(random(150),random(150),random(150));
        neoBackLeft(random(150),random(150),random(150));
        neoBackRight(random(150),random(150),random(150));
        // Play song
        timerMarioMelody = millis() + playMario();
      }
    }
  }
  else
  {
    if (readBlackLine())
    {
      breakWheels();
      analogWrite(PIN_LEFT_WHEEL_FORWARD, BASE_DRIVE_SPEED);
      analogWrite(PIN_RIGHT_WHEEL_FORWARD, BASE_DRIVE_SPEED);
      resetWheelCounters();
      while (wheelSensorCounter < INTERSECTION_CHECK_DRIVE_DISTANCE)
      {
        readLine();
        readWheels();
      }
      if (readBlackLine())
      {
        // Ending sequence
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
        rotateWheels(true, BASE_ROTATION_SPEED, 0);
      }
    }
    else if (readRightLine())
    {
      rotateWheels(true, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
    }
    else if (readLeftLine() && !readStraightLine())
    {
      rotateWheels(false, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
    }
    else if (readWhiteLine())
    {
      rotateWheels(true, BASE_ROTATION_SPEED, ROTATION_CORRECTION_DRIVE_DISTANCE);
    }
    else
    {
      drivePID(BASE_DRIVE_SPEED);
    }
  }
}

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                      * Methods *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
// Go through all sensors and see if they all see black
bool readBlackLine()
{
  for (int i = 0; i < 8; i++)
  {
    if (sensors[i] < qtr.calibrationOn.maximum[i] - CALIBRATION_CORRECTION_VALUE)
    {
      return false;
    }
  }
  return true;
}

// Go through all sensors and see if they all see white
bool readWhiteLine()
{
  for (int i = 0; i < 8; i++)
  {
    if (sensors[i] > qtr.calibrationOn.minimum[i] + CALIBRATION_CORRECTION_VALUE)
    {
      return false;
    }
  }
  return true;
}

// Go through the right range of sensors and see if any of them see black
bool readRightLine()
{
  for (int i = RIGHT_RANGE_MIN; i < RIGHT_RANGE_MAX; i++)
  {
    if (sensors[i] >= qtr.calibrationOn.maximum[i] - CALIBRATION_CORRECTION_VALUE)
    {
      return true;
    }
  }
  return false;
}

// Go through the left range of sensors and see if any of them see black
bool readLeftLine()
{
  for (int i = LEFT_RANGE_MIN; i < LEFT_RANGE_MAX; i++)
  {
    if (sensors[i] >= qtr.calibrationOn.maximum[i] - CALIBRATION_CORRECTION_VALUE)
    {
      return true;
    }
  }
  return false;
}

// Go through the range between the left range and right range of sensors and see if they all see black
bool readStraightLine()
{
  for (int i = LEFT_RANGE_MAX; i < RIGHT_RANGE_MIN; i++)
  {
    if (sensors[i] >= qtr.calibrationOn.maximum[i] - CALIBRATION_CORRECTION_VALUE)
    {
      return true;
    }
  }
  return false;
}

// Use the sensors to read the line
void readLine()
{
  lineReadData = qtr.readLineBlack(sensors);
}

// Use pulse sensor to see how far the right wheel has turned
void readWheels()
{
  if (analogRead(PIN_RIGHT_WHEEL_SENSOR) != wheelSensorCounterLast)
  {
    wheelSensorCounter++;
    wheelSensorCounterLast = analogRead(PIN_RIGHT_WHEEL_SENSOR);
  }
}

// Reset the pulse counter
void resetWheelCounters()
{
  wheelSensorCounter = 0;
  wheelSensorCounterLast = 0;
}

// Stop movement
void breakWheels()
{
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, 0);
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, 0);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, 0);
}

// Forward
void driveForward(unsigned short speed)
{ 
  breakWheels();
  analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
}

// Backward
void driveBackward(unsigned short speed)
{
  breakWheels();
  analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
}

// Rotate the bot by 45 degrees, then keep rotating until it sees a line in the middle
void rotateWheels(bool goRight, int speed, int compensationDistance)
{
  short degrees = ((goRight) ? 45 : -45);
  float degreesLeft = WHEEL_ROTATION_TICK_DEGREES * abs(degrees);
  // Break
  resetWheelCounters();
  breakWheels();
  // Drive forward to compensate for lost progression when turning
  driveForward(speed);
  while (wheelSensorCounter < compensationDistance)
  {
    readWheels();
  }
  resetWheelCounters();
  // Break
  breakWheels();
  neo.clear();
  // Decide what to do with the lights and wheels depending on the direction
  if (goRight)
  {
    neoLeft(20,5,0);
    neoRight(0,100,30);
    analogWrite(PIN_LEFT_WHEEL_FORWARD, speed);
    analogWrite(PIN_RIGHT_WHEEL_BACKWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
  }
  else
  {
    neoLeft(0,100,30);
    neoRight(20,5,0);
    analogWrite(PIN_RIGHT_WHEEL_FORWARD, speed * RIGHT_WHEEL_CORRECTION_MULTIPLIER);
    analogWrite(PIN_LEFT_WHEEL_BACKWARD, speed);
  }
  // Rotate at least 45 degrees
  while (wheelSensorCounter < degreesLeft)
  {
    readWheels();
  }
  // Keep going until we reach a line at the center of the sensors
  readLine();
  while (!readStraightLine())
  {
    readLine();
  }
  // Stop and wait a little bit before continuing
  breakWheels();
  for (int i = 0; i < 10; i++)
  {
    readLine();
    delay(10);    
  }
}

// Use PID to drive smoothly
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
  m1Speed = min(max(m1Speed, 0), speed);
  m2Speed = min(max(m2Speed, 0), speed);

  // Stop if we try to turn too much
  if (abs(m1Speed - m2Speed) > MAX_STRAIGHT_DIFFERENCE) { return; }

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
  return duration / 2 * 0.034; // 0.034 = speed of sound in air at room temperature in cm/sec
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