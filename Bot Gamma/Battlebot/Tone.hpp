#include "Notes.hpp"

int tonePin = 8;

void setupTone()
{
  pinMode(tonePin, OUTPUT);
}

void playTone(int loopCounter)
{
  tone(tonePin, NOTE_D5);
}