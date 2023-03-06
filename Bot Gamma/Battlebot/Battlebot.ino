// Libraries
#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>


// Gripper
const int gripper = 10;
const int closedAngle = 180;
const int openedAngle = 130;

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
const int rotationSpeed = 160; // Speed at which to rotate
const int driveSpeed = 160; // Speed at which to drive
int actualSpeed = 255; // The currently set speed to the motors

// Sensors
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData; // Value depending on where the sensor is detecting a line
                  // A value closer to 0 is more left; closer to 7000 is more right; 3500 is center
                  // 0 means no line is detected; 7000 means everything is detecting.

// Sensor Calibration
const int calibrationTime = 250; // in milliseconds * 20 (50 = 1 second)
const bool shouldCalibrate = true;

// Loop Counter
int loopCounter;

//rotatedLeftLast
bool rotatedLeftLast;

// Please remove
bool haveFun = false;


// Bluetooth
SoftwareSerial configureBT(2, 4); // TX || RX

// NEO PIXELS
Adafruit_NeoPixel neoPixel(4, 7, NEO_GRB + NEO_KHZ800);


void setup()
{    
  // Bluetooth setup 
  Serial.begin(9600);
  configureBT.begin(38400); 

  // NEO setup
  neoPixel.begin();
  
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
      neoPixel.clear();
      neoFrontLeft(random(150),random(150),random(150));
      neoFrontRight(random(150),random(150),random(150));
      neoBackLeft(random(150),random(150),random(150));
      neoBackRight(random(150),random(150),random(150));
      driveBreak(false);
      if (i % 10 == 0)
      {
        (i % 20 == 0) ? rotateLeft(false) : rotateRight(false);
      }
      else if (i % 15 == 0)
      {
        (i % 30 == 0) ? rotateLeft(false) : rotateRight(false); 
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
    driveRightWheel();
    delay(20);
  }
  else
  {
  // NEO clear
  neoClear();
    
  // Serial monitor
  if (configureBT.available()) {
    Serial.write(configureBT.read());
  }
  
  if (Serial.available()) {
    configureBT.write(Serial.read());
  }
  
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
    analogWrite(leftWheelFwd, getFactor(lineReadData, false));
    analogWrite(leftWheelBwd, 0);
    analogWrite(rightWheelFwd, getFactor(lineReadData, true));
    analogWrite(rightWheelBwd, 0);
    if (lineReadData < 7000) { rotatedLeftLast = lineReadData < 3500; }
  }
  else // Cannot detect any lines
  {
    // Prefer left over right, so rotate right.
    actualSpeed = rotationSpeed;
    (rotatedLeftLast ? rotateLeft(true) : rotateRight(true));
  }
 
  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

  // Increment loop counter
  loopCounter += 1;
  }
}

int getFactor(int line, bool is_left)
{
  if ((is_left && line <= 1500) || (!is_left && line >= 5500))
  {
    return 255;
  }
  else if ((is_left && line <= 3500) || (!is_left && line >= 3500))
  {
    return 220;
  }
  else if ((is_left && line <= 4500) || (!is_left && line >= 2500))
  {
    return 150;
  }
  else if ((is_left && line <= 5000) || (!is_left && line >= 2000))
  {
    return 120;
  }
  else if ((is_left && line <= 5500) || (!is_left && line >= 1500))
  {
    return 100;
  }
  else if ((is_left && line <= 6000) || (!is_left && line >= 1000))
  {
    return 80;
  }
  else if ((is_left && line <= 6500) || (!is_left && line >= 500))
  {
    return 50;
  }
  else
  {
    return 20;
  }
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
void driveFwd(bool doLights)
{
  if (doLights)
  {
    neoBack(50, 0, 0);
    neoFront(50, 50, 50);
  }
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

void reverseLeftWheel()
{
  analogWrite(leftWheelFwd, 0);
  analogWrite(leftWheelBwd, actualSpeed);
}

void reverseRightWheel()
{
  analogWrite(rightWheelFwd, 0);
  analogWrite(rightWheelBwd, actualSpeed);
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
void rotateRight(bool doLights)
{
  if (doLights)
  {
    neoRight(0, 0, 0);
    neoLeft(150, 50, 0);
  }
  driveBreak(false);
  driveLeftWheel();
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
  driveRightWheel();
  reverseLeftWheel();
}

// Neo Pixel
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

void neoBackLeft(int r, int g, int b)
{
  neoPixel.setPixelColor(1, neoPixel.Color(g, r, b));
  neoPixel.show();
}

void neoBackRight(int r, int g, int b)
{
  neoPixel.setPixelColor(0, neoPixel.Color(g, r, b));
  neoPixel.show();
}

void neoFrontLeft(int r, int g, int b)
{
  neoPixel.setPixelColor(2, neoPixel.Color(g, r, b));
  neoPixel.show();
}

void neoFrontRight(int r, int g, int b)
{
  neoPixel.setPixelColor(3, neoPixel.Color(g, r, b));
  neoPixel.show();
}

void neoClear()
{
  neoPixel.clear();
  return;
  neoPixel.setPixelColor(0, neoPixel.Color(0, 0, 0));
  neoPixel.setPixelColor(1, neoPixel.Color(0, 0, 0));
  neoPixel.setPixelColor(2, neoPixel.Color(0, 0, 0));
  neoPixel.setPixelColor(3, neoPixel.Color(0, 0, 0));
  neoPixel.show();
}
