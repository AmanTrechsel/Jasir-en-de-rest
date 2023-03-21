#include <SoftwareSerial.h>
#include "NeoPixel.hpp"

const int txPin = 2;
const int rxPin = 3;

SoftwareSerial bluetoothSerial(rxPin, txPin);

void setupBluetooth()
{
  bluetoothSerial.begin(9600);
  bool canStart = false;
  int i = 0;
  while (!canStart)
  {
    // Since we don't have working Bluetooth just set it to true after 1000 tries
    if (i >= 1000)
    {
      canStart = true;
    }
    // Bluetooth check
    if (bluetoothSerial.available() > 0)
    {
      canStart = bluetoothSerial.read();
      if (i%10 == 0)
      {
        neoClear();
        switch (i%4)
        {
          case 0:
            neoFrontLeft(0,10,30);
            break;
          case 1:
            neoFrontRight(0,10,30);
            break;
          case 2:
            neoBackLeft(0,10,30);
            break;
          case 3:
            neoBackRight(0,10,30);
            break;
        }
      }
      i++;
    }
  }
  
}