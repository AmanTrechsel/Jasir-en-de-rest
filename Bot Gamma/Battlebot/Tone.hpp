#include "Notes.hpp"

const int tonePin = 8;
bool currentTone;

int melody[] = {
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_C3, NOTE_C4, NOTE_G3, NOTE_E3, NOTE_C4, NOTE_G3, NOTE_E3,
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_DS4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_B4
};

int noteDurations[] = {
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  32,32,16,32,32,16,32,32,16,8
};

int melody2[] = {
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_DS4, NOTE_C5, 0, NOTE_C5, NOTE_DS4, NOTE_C5, NOTE_DS4, NOTE_C5,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C3, NOTE_B4, 0, NOTE_B4, NOTE_C3, NOTE_B4, NOTE_C3, NOTE_B4
};

int noteDurations2[] = {
  8,8,8,16,16,16,8,16,16,16,8,
  8,8,8,16,16,16,8,16,16,16,8
};

void playFinish(int loopCounter)
{
  for (int i = 0; i < 22*4; i++)
  {
    i = i%23;
    int noteDuration = 1000 / noteDurations2[i];
    tone(tonePin, melody2[i], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(tonePin);
  }
  noTone(tonePin);
}

void setupTone()
{
  pinMode(tonePin, OUTPUT);
  for (int i = 0; i < 31; i++)
  {
    int noteDuration = 1000 / noteDurations[i] * 1.5;
    tone(tonePin, melody[i], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(tonePin);
  }
  noTone(tonePin);
  playFinish(1);
}

void playTone(int loopCounter)
{
  noTone(tonePin);
  if (loopCounter%10 == 0)
  {
    if (currentTone)
    {
      tone(tonePin, NOTE_FS3, 125);
    } 
    else
    {
      tone(tonePin, NOTE_FS2, 125);
    } 
    currentTone = !currentTone;  
  }
}