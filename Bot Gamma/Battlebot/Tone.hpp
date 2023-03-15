#include "Notes.hpp"

const int tonePin = 8;
bool currentTone;

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
  tone(tonePin, NOTE_FS3);
  delay(300);
  tone(tonePin, NOTE_E3);
  delay(200);
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
  tone(tonePin, NOTE_FS3);
  delay(300);
  tone(tonePin, NOTE_E3);
  delay(200);
  noTone(tonePin);
}

void playTone(int loopCounter)
{
  if (loopCounter%10 == 0)
  {
    noTone(tonePin);
    if (currentTone)
    {
      tone(tonePin, NOTE_F2);
    } 
    else
    {
      tone(tonePin, NOTE_B2);
    } 
    currentTone = !currentTone;  
  }
}