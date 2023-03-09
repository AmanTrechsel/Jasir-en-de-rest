// Libraries
#include <Adafruit_NeoPixel.h>

// NEO PIXELS
Adafruit_NeoPixel neoPixel(4, 7, NEO_GRB + NEO_KHZ800);

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
