// Libraries
#include "Tone.hpp"

// Timers
int loopCounter;
unsigned long funTimer;
unsigned long funSongTimer;
unsigned long blackLineTimer;
unsigned long rotateTimer;

// States
bool haveFun = false;
bool seenBlack = false;
bool avoidingObstacle = false;
bool rotating = false;

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

  delay(ROTATE_DELAY);
  while (lineReadData > MIN_CENTER_VALUE && lineReadData < MAX_CENTER_VALUE)
  {
    // Check the sensors and output the values
    readLine();
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
    int turnAmt = abs(loopCounter / TURN_DISTANCE * 3500);
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
      loopCounter = -TURN_DISTANCE;
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
            rotateRight2();
            rotateTimer = ROTATE_DELAY + millis();
            rotating = true;
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
      if (sensors[5] >= BLACK_THRESHOLD || sensors[6] >= BLACK_THRESHOLD || sensors[7] >= BLACK_THRESHOLD)
      {
        rotateRight2();
        rotateTimer = ROTATE_DELAY + millis();
        rotating = true;
      }      
    }
    
    neoFull(random(150),random(150),random(150));
    if (!rotating || rotateTimer <= millis() || (lineReadData != 0 && rotateTimer - ROTATE_DELAY/2 <= millis()))
    {
      driveAdvanced(lineReadData - 3500);
      rotating = false;
    }
    readRightWheelSensor();
  }
  // Increment loop counter
  loopCounter = (loopCounter+1)%10000;
}
