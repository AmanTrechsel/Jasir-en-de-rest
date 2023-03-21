// Libraries
#include "Tone.hpp"

// Timers
int loopCounter;
int funTimer;
int funSongTimer;
int blackLineTimer;

// States
bool haveFun = false;
bool seenBlack = false;
bool avoidingObstacle = false;

void setup()
{    
  // Setup all included scripts
  setupWheels();
  setupNeoPixel();
  setupGripper();
  setupBluetooth();
  setupTone();

  openGripper();
  actualSpeed = 255;
  driveFwd(true);
  delay(50);
  actualSpeed = 180;
  driveFwd(true);

  setupSensors();
  setupEcho();

  closeGripper();
  delay(10);
  driveBreak(true);
  rotateLeft(true);

  while (wheelSensorCounter < 30)
  {
    readRightWheelSensor();
    delay(10);
  }
  driveBreak(true);
  driveFwd(true);
  delay(500);
  
  driveBreak(false);
}

void loop()
{
  // NEO clear
  neoClear();

  if (haveFun)
  {
    if (millis() >= funTimer)
    {
      neoFrontLeft(random(150),random(150),random(150));
      neoFrontRight(random(150),random(150),random(150));
      neoBackLeft(random(150),random(150),random(150));
      neoBackRight(random(150),random(150),random(150));
      actualSpeed = 255;
      reverseLeftWheel();
      driveRightWheel(actualSpeed);
      funTimer = millis() + 100;
    }
    if (millis() >= funSongTimer)
    {
      funSongTimer = millis() + playFinish();
    }
  }
  else if (avoidingObstacle)
  {
    int turnAmt = loopCounter / TURN_DISTANCE * 3500;
    driveAdvanced(turnAmt - 3500);
    if (loopCounter >= TURN_DISTANCE * 2) { avoidingObstacle = false; }
  }
  else
  {
    playTone(loopCounter);

    // Avoid obstacles if anything is in front of it
    if (getDistance() <= OBSTACLE_DISTANCE)
    {
      avoidingObstacle = true;
      loopCounter = -1;
    }

    // Check the sensors and output the values
    readLine();

    if (readBlackLine())
    {    
      if (seenBlack)
      {
        if (blackLineTimer < millis())
        {
          seenBlack = false;
          if (readBlackLine())
          {
            neoFull(128,128,128);
            driveBreak(false);
            openGripper();
            delay(400);
            reverseLeftWheel();
            reverseRightWheel();
            delay(400);
            driveBreak(false);
            delay(400);
            haveFun = true;
            playFinish();
          }
          else
          {
            rotateRight(false);
          }
        }
      }
      else
      {
        seenBlack = true;
        blackLineTimer = BLACK_LINE_TIMEOUT + millis();
      }
    }
    else
    {
      neoFull(random(150),random(150),random(150));
      driveAdvanced(lineReadData - 3500);
    }
    readRightWheelSensor();
  }
  // Increment loop counter
  loopCounter += 1;
}
