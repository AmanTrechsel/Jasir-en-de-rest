#ifndef LightShow_H
#define LightShow_H

// Libraries
#include <Adafruit_NeoPixel.h>

// Led 
Adafruit_NeoPixel neoPixel(4, 8, NEO_GRB + NEO_KHZ800);

void setupNeoPixel()
{    
  // NEO setup
  neoPixel.begin();
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

// Neo Pixel
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

 void party()
 {
      neoFrontLeft(random(150),random(150),random(150));
      neoFrontRight(random(150),random(150),random(150));
      neoBackLeft(random(150),random(150),random(150));
      neoBackRight(random(150),random(150),random(150));
 }
#endif
