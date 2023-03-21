// Libraries
#include <QTRSensors.h>
#include "Wheels.hpp"

QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;

void readLine()
{
  lineReadData = qtr.readLineBlack(sensors);
}

void setupSensors()
{    
  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  openGripper();
    
  // Calibration
  while (wheelSensorCounter < CALIBRATION_DISTANCE)
  {
    neoPixel.clear();
    neoFrontLeft(random(150),random(150),random(150));
    neoFrontRight(random(150),random(150),random(150));
    neoBackLeft(random(150),random(150),random(150));
    neoBackRight(random(150),random(150),random(150));
    qtr.calibrate();
    readLine();
    readRightWheelSensor();
  }
  closeGripper();
  wheelSensorCounter = 0;
  actualSpeed = 255;
}

bool readBlackLine()
{
  return (sensors[0] > BLACK_THRESHOLD) && (sensors[1] > BLACK_THRESHOLD) && (sensors[2] > BLACK_THRESHOLD) && (sensors[3] > BLACK_THRESHOLD) && (sensors[4] > BLACK_THRESHOLD) && (sensors[5] > BLACK_THRESHOLD) && (sensors[6] > BLACK_THRESHOLD) && (sensors[7] > BLACK_THRESHOLD);
}
