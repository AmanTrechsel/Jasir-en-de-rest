// Libraries
#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Gripper
const int gripper = 10;
const int closedAngle = 250;
const int openedAngle = 1400;

// Clicker
const int echo = 12; // Speaker
const int trig = 13; // Microphone
long duration;
int distance;

// Motor
const int leftWheelFwd = 11;
const int leftWheelBwd = 6;
const int rightWheelBwd = 5;
const int rightWheelFwd = 3;
const int rotationSpeed = 150; // Speed at which to rotate
const int driveSpeed = 150; // Speed at which to drive
int actualSpeed = 255; // The currently set speed to the motors

// Sensors
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData; // Value depending on where the sensor is detecting a line
                  // A value closer to 0 is more left; closer to 7000 is more right; 3500 is center
                  // 0 means no line is detected; 7000 means everything is detecting.

// Sensor Calibration
const int calibrationTime = 250; // in milliseconds * 20 (50 = 1 second)
const bool shouldCalibrate = false;

// Loop Counter
int loopCounter;

void setup()
{  
  // Echo locator
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  
  // Initiate Serial
  Serial.begin(9600);

  // Sensor Calibration
  if (shouldCalibrate)
  {
    int i;
    Serial.println("");
    Serial.print("Calibrating");
    for (i = 0; i < calibrationTime; i++)
    {
      driveBreak();
      if (i % 10 == 0)
      {
        (i % 20 == 0) ? rotateLeft() : rotateRight();
      }
      else if (i % 15 == 0)
      {
        (i % 30 == 0) ? rotateLeft() : rotateRight(); 
      }
      if ((i % 100 == 0 || i == 150) && i > 0)
      {
        Serial.print(".");
      }
      qtr.calibrate();
      delay(20);
    }
    Serial.println("");
    Serial.println("Calibration complete");
  }

  // Initiate Gripper
  pinMode(gripper, OUTPUT);

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
  // Start Debug line
  Serial.println("===============================================================");
  Serial.println("");

  // Get the distance to anything in front of it
  distance = getDistance();
  
  // Output Distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

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
  for (uint16_t i = 0; i < 8; i++)
  {
    Serial.print(sensors[i]);
    Serial.print("\t");
    if (i == 7) { Serial.println(""); }
  }
 
  // Control Wheels based on Distance
  if (lineReadData > 0)
  {
    if (lineReadData <= 2500)
    {
      actualSpeed = rotationSpeed;
      rotateLeft();
    }
    else if (lineReadData >= 4500)
    {
      actualSpeed = rotationSpeed;
      rotateRight();
    }
    else
    {
      actualSpeed = driveSpeed;
      driveFwd();
    }
  }
  else // Cannot detect any lines
  {
    // Prefer left over right, so rotate right.
    actualSpeed = rotationSpeed;
    rotateLeft();
  }
 
  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

  // Increment loop counter
  loopCounter += 1;
}


// Calculate Distance
int getDistance()
{
  // Echo Locator
  digitalWrite(trig, LOW);
  delayMicroseconds(20);
  digitalWrite(trig, HIGH);
  delayMicroseconds(100);
  digitalWrite(trig, LOW);

  // Receive Distance
  duration = pulseIn(echo, HIGH);
  return duration / 2 * 0.034;
}

// Gripper
void openGripper()
{
  analogWrite(gripper, openedAngle);
}

void closeGripper()
{
  analogWrite(gripper, closedAngle);
}

// Forward
void driveFwd()
{
  driveLeftWheel();
  driveRightWheel();
}

void driveLeftWheel()
{
  analogWrite(leftWheelBwd, 0);
  analogWrite(leftWheelFwd, actualSpeed);
}

void driveRightWheel()
{
  analogWrite(rightWheelBwd, 0);
  analogWrite(rightWheelFwd, actualSpeed);
}

// Backwards
void driveBwd()
{
  reverseLeftWheel();
  reverseRightWheel();
}

void reverseLeftWheel()
{
  analogWrite(leftWheelBwd, actualSpeed);
}

void reverseRightWheel()
{
  analogWrite(rightWheelBwd, actualSpeed);
}

// Breaking
void driveBreak()
{
  breakLeftWheel();
  breakRightWheel();
}

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

// Rotation
void rotateRight()
{
  driveBreak();
  driveLeftWheel();
  reverseRightWheel();
}

void rotateLeft()
{
  driveBreak();
  driveRightWheel();
  reverseLeftWheel();
}
