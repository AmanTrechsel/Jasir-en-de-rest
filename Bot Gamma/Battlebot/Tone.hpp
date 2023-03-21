#include "Sensors.hpp"
#include "Notes.hpp"

const int tonePin = 8;
int songPosition;
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

int playFinish()
{
  noTone(tonePin);
  songPosition = songPosition%23;
  int noteDuration = 1000 / noteDurations2[songPosition];
  tone(tonePin, melody2[songPosition], noteDuration);
  int pauseBetweenNotes = noteDuration * 1.30;
  songPosition++;
  return pauseBetweenNotes;
}

void setupTone()
{
  pinMode(tonePin, OUTPUT);
  for (int i = 0; i < 31; i++)
  {
    neoPixel.clear();
    if (i < 21)
    {
      switch (i%4)
      {
        case 0:
          neoFrontLeft(70,50,0);
          break;
        case 1:
          neoFrontRight(70,50,0);
          break;
        case 2:
          neoBackLeft(70,50,0);
          break;
        case 3:
          neoBackRight(70,50,0);
          break;
      }
    }
    else
    {
      if (i%2 == 0)
      {
        neoFrontLeft(150,120,0);
        neoFrontRight(150,120,0);
      }
      else
      {
        neoBackLeft(150,120,0);
        neoBackRight(150,120,0);
      }
    }
    int noteDuration = 1000 / noteDurations[i] * 1.5;
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
  if (loopCounter%20 == 0)
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