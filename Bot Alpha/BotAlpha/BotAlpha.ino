// Libraries
#include <QTRSensors.h>

// Constants
const int leftWheelFwd = 11; // Links, vooruit
const int leftWheelBwd = 10; // Links, achteruit
const int rightWheelFwd = 9; // Rechts, achteruit
const int rightWheelBwd = 6; // Rechts, vooruit

// PID constants
const float KP = 0.225;
const float KD = 2.25;

int lastError = 0;

// Base motor speed
const int M1 = 255;
const int M2 = 255;

// Sensor Calibration
const int calibrationTime = 20; // in milliseconds * 20 (50 = 1 second)
const bool shouldCalibrate = true;

// Sensors
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() 
{
  // Sensors configuration
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  // Sensor Calibration
  if (shouldCalibrate)
  {
    int i;
    for (i = 0; i < calibrationTime; i++)
    {
      slowGoForward();
      delay(10);
      qtr.calibrate();
      delay(10);
    }
  }

  // Wheels configuration
  pinMode(leftWheelFwd, OUTPUT);
  pinMode(leftWheelBwd, OUTPUT);
  pinMode(rightWheelFwd, OUTPUT);
  pinMode(rightWheelBwd, OUTPUT);
}

void loop()
{
  // Read sensor
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculating turns
  int error = position - 3500;
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  // Calculating motor speeds
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;

  // Min and max speeds 
  m1Speed = min(max(m1Speed, 0), 255);
  m2Speed = min(max(m2Speed, 0), 255);

  // Let bot drive
  analogWrite(leftWheelFwd, m1Speed);
  analogWrite(rightWheelFwd, m2Speed);

  // Stop when all sensors detect black
  if((sensorValues[0] > 980) && (sensorValues[1] > 980) && (sensorValues[2] > 980) && (sensorValues[3] > 980) && (sensorValues[4] > 980) && (sensorValues[5] > 980) && (sensorValues[6] > 980) && (sensorValues[7] > 980))
  {
    analogWrite(leftWheelFwd, 0);
    analogWrite(rightWheelFwd, 0);
  }
}

void slowGoForward()
{
  analogWrite(leftWheelFwd, 200);
  analogWrite(rightWheelFwd, 200);
}
