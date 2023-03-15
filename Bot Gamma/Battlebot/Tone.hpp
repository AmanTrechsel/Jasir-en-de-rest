#include "Notes.hpp"

const int tonePin = 8;

void setupTone()
{
  pinMode(tonePin, OUTPUT);
  tone(tonePin, NOTE_B4);
  delay(100);
  tone(tonePin, NOTE_B5);
  delay(100);
  tone(tonePin, NOTE_FS4);
  delay(100);
  tone(tonePin, NOTE_DS4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
  tone(tonePin, NOTE_F4);
  delay(100);
}

void playTone(int loopCounter)
{
  
}