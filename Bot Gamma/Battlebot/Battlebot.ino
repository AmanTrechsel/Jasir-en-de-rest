// Libraries
#include "Sensors.hpp"

// Loop Counter
int loopCounter;

// Please remove
bool haveFun = false;

void setup()
{    
  
  // Setup all included scripts
  setupWheels();
  setupNeoPixel();
  setupGripper();

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
  // Clear history
  clearHistory();
}

void loop()
{
  // Have fun toggle
  if (digitalRead(8) == LOW)
  {
    haveFun = !haveFun;
  }

  if (haveFun)
  {
    neoPixel.clear();
    neoFrontLeft(random(150),random(150),random(150));
    neoFrontRight(random(150),random(150),random(150));
    neoBackLeft(random(150),random(150),random(150));
    neoBackRight(random(150),random(150),random(150));
    actualSpeed = 255;
    reverseLeftWheel();
    driveRightWheel(actualSpeed);
    delay(100);
  }
  else
  {
    // NEO clear
    neoClear();
  }

  // Get the distance to anything in front of it
  distance = getDistance();

  // Check the sensors and output the values
  readLine();

  if (hasSeenMostlyBlack())
  { 
    if ((sensors[0] > 980) && (sensors[1] > 980) && (sensors[2] > 980) && (sensors[3] > 980) && (sensors[4] > 980) && (sensors[5] > 980) && (sensors[6] > 980) && (sensors[7] > 980))
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
    }
    else
    {
      rotateRight(true);
    }
  }
  else
  {
    neoFull(random(150),random(150),random(150));
    // Calculating turns
    int error = lineReadData - 3500;

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

  readRightWheelSensor();
 
 
  // Increment loop counter
  loopCounter += 1;
}
