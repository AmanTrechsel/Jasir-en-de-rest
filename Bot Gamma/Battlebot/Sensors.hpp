// Libraries
#include <QTRSensors.h>
#include "Wheels.hpp"

// Sensors
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;
int calibrationTime = 3;

void setupSensors()
{    
  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
    
  // Calibration
  int i;
  for (i = 0; i < calibrationTime; i++)
  {
    neoPixel.clear();
    neoFrontLeft(random(150),random(150),random(150));
    neoFrontRight(random(150),random(150),random(150));
    neoBackLeft(random(150),random(150),random(150));
    neoBackRight(random(150),random(150),random(150));
    qtr.calibrate();
    delay(100);
  }
}