// Libraries
#include <QTRSensors.h>
#include "Wheels.hpp"

const int LINE_HISTORY_LENGTH = 100;
const int ERROR_THRESHOLD = 5;
QTRSensors qtr;
uint16_t sensors[8];
int lineReadData;
int* lineReadDataHistory;

int lineHistoryTotal()
{
  int totalLineHistory = 0;
  for (int i = LINE_HISTORY_LENGTH; i > 0; i--) {
    totalLineHistory += lineReadDataHistory[i];
  }
  return totalLineHistory;
}

void readLine()
{
  lineReadData = qtr.readLineBlack(sensors);
  for (int i = LINE_HISTORY_LENGTH; i > 0; i--) {
    lineReadDataHistory[i] = lineReadDataHistory[i-1];
  }
  lineReadDataHistory[0] = lineReadData;
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

int blackHistory()
{
  return 7000 * (ERROR_THRESHOLD / 100) * LINE_HISTORY_LENGTH;
}

bool hasSeenMostlyBlack()
{
  return lineHistoryTotal() >= blackHistory();
}


void setupSensors()
{    
  // Line Sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6,A0,A7,A1,A2,A3,A4,A5},8);
  lineReadDataHistory = new int[LINE_HISTORY_LENGTH];
    
  // Calibration
  int total = lineHistoryTotal();
  int blackHistory = blackHistory();
  int i;
  while (total < blackHistory)
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
