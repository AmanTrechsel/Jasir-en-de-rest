// Libraries
#include <SoftwareSerial.h>
#include <QTRSensors.h>

// Motor
const int leftWheelFwd = 11; // Links, vooruit
const int leftWheelBwd = 10; // Links, achteruit
const int rightWheelFwd = 9; // Rechts, achteruit
const int rightWheelBwd = 6; // Rechts, vooruit
const int rotationSpeed = 110; // Speed at which to rotate
const int driveSpeed = 180; // Speed at which to drive
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

// Bluetooth configuration
SoftwareSerial configureBT(3, 2); // TX || RX, in feite zijn dit de pinmodes

void setup() {
  // Bluetooth setup 
  Serial.begin(9600);
  configureBT.begin(9600); // Indien je commands wilt uitvoeren, knop op module ingedrukt houden bij het inpluggen en 9600 aanpassen naar 38400

    // Wheels
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);

  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7},8);
  
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
}

void loop() { 
  if (configureBT.available()) {
    Serial.write(configureBT.read());
  }
  
  if (Serial.available()) {
    configureBT.write(Serial.read());
  }

    // Start Debug line
  Serial.println("===============================================================");
  Serial.println("");

  // Divide Debug line
  Serial.println("-----------------------------------------------------");
  Serial.println("");

  
  // Check the sensors and output the values
  lineReadData = qtr.readLineBlack(sensors);
  for (uint16_t i = 0; i < 8; i++)
  {
    Serial.print(sensors[i]);
    Serial.print("\t");
    if (i == 7) { Serial.println(lineReadData); }
  }
 
  // Control Wheels based on Distance
  if (lineReadData > 0)
  {
    if (lineReadData <= 2400)
    {
      actualSpeed = rotationSpeed;
      rotateLeft();
    }
    else if (lineReadData >= 4300)
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
