// Libraries
#include <QTRSensors.h>
#include "Wheels.hpp"

const int LINE_HISTORY_LENGTH = 500;
const int ERROR_THRESHOLD = 5
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;
int calibrationTime = 3;
int* lineReadDataHistory;

void setupSensors()
{    
  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  lineReadDataHistory = new int[LINE_HISTORY_LENGTH];
    
  // Calibration
  int i;
  while (lineHistoryTotal() < 3300000)
  {
    neoPixel.clear();
    neoFrontLeft(random(150),random(150),random(150));
    neoFrontRight(random(150),random(150),random(150));
    neoBackLeft(random(150),random(150),random(150));
    neoBackRight(random(150),random(150),random(150));
    qtr.calibrate();
    readLine();
    delay(100);
  }
}

void readLine()
{
  lineReadData = qtr.readLineBlack(sensors);
  for (int i = LINE_HISTORY_LENGTH; i > 0; i--) {
    lineReadDataHistory[i] = lineReadDataHistory[i-1];
  }
  lineReadDataHistory[0] = lineReadData;
}

void lineHistoryTotal()
{
  int totalLineHistory = 0;
  for (int i = LINE_HISTORY_LENGTH; i > 0; i--) {
    totalLineHistory += lineReadDataHistory[i];
  }
  return totalLineHistory;
}

int historyCount(int value)
{
  int count = 0;
  for (int i = 0; i < LINE_HISTORY_LENGTH; i++)
  {
    if (lineReadDataHistory[i] == value)
    {
      count++;
    }
  }
  return count;
}

bool hasSeenMostlyBlack()
{
  return lineHistoryTotal() * 7000 * (ERROR_THRESHOLD / 100) >= LINE_HISTORY_LENGTH
}