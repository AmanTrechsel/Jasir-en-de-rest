#include "Notes.hpp"

const int tonePin = 8;

void setupTone()
{
  pinMode(tonePin, OUTPUT);
  tone(tonePin, NOTE_B3);
  delay(100);
  tone(tonePin, NOTE_B4);
  delay(100);
  tone(tonePin, NOTE_FS3);
  delay(100);
  tone(tonePin, NOTE_DS3);
  delay(100);
  tone(tonePin, NOTE_B4);
  delay(50);
  tone(tonePin, NOTE_FS4);
  delay(300);
  tone(tonePin, NOTE_F3);
  delay(100);
  tone(tonePin, NOTE_F3);
  delay(100);
  tone(tonePin, NOTE_F3);
  delay(100);
  tone(tonePin, NOTE_F3);
  delay(100);
}

void playTone(int loopCounter)
{
  
}