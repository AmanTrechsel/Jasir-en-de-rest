#include "Notes.hpp"

const int tonePin = 8;
bool currentTone;

int melody[] = {
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_C3, NOTE_C4, NOTE_G3, NOTE_E3, NOTE_C4, NOTE_G3, NOTE_E3,
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_DS4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_GS4, NOTE_A5, NOTE_B5
};

int noteDurations[] = {
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  32,32,16,32,32,16,32,32,16,8
};

void setupTone()
{
  pinMode(tonePin, OUTPUT);
  for (int i = 0; i < 31; i++)
  {
    int noteDuration = 1000 / noteDurations[i];
    tone(tonePin, melody[i], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(tonePin);
  }
  noTone(tonePin);
}

void playTone(int loopCounter)
{
  noTone(tonePin);
  if (loopCounter%10 == 0)
  {
    if (currentTone)
    {
      tone(tonePin, NOTE_FS5, 125);
    } 
    else
    {
      tone(tonePin, NOTE_FS4, 125);
    } 
    currentTone = !currentTone;  
  }
}