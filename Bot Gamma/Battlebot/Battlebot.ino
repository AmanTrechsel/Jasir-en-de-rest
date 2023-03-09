// Libraries
#include "Sensors.hpp"

// Loop Counter
int loopCounter;

// Please remove
bool haveFun = false;

void setup()
{    
  driveBreak(false);
  rotateLeft(true);
  delay(500);
  driveFwd(true);
  delay(250);

  // Setup all included scripts
  setupWheels();
  setupNeoPixel();
  setupSensors();
  setupGripper();
  setupEcho();

  // Start 'Animation'
  delay(150);
  openGripper();
  delay(150);
  closeGripper();
  delay(150);
  openGripper();
  delay(150);
  closeGripper();
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
  

  // Control Gripper based on Distance
  if (distance < 5)
  {
    openGripper();
  }
  else
  {
    closeGripper();
  }

  // Check the sensors and output the values
  lineReadData = qtr.readLineBlack(sensors);

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
  analogWrite(leftWheelFwd, m1Speed);
  analogWrite(rightWheelFwd, m2Speed);

  // finish line
  if ((sensors[0] > 980) && (sensors[1] > 980) && (sensors[2] > 980) && (sensors[3] > 980) && (sensors[4] > 980) && (sensors[5] > 980) && (sensors[6] > 980) && (sensors[7] > 980))
  {
    driveBreak(true);
    delay(400);
    reverseLeftWheel();
    reverseRightWheel();
    delay(400);
    driveBreak(true);
    delay(400);
    haveFun = true;
  }
  else
  {
    analogWrite(leftWheelFwd, m1Speed);
    analogWrite(rightWheelFwd, m2Speed);
  }
 
 
  // Increment loop counter
  loopCounter += 1;
}