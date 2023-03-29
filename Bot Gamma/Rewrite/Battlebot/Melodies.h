#include "Notes.h"

const int PIN_TONE = 0;
const int PACMAN_NOTES = 32;
const int PACMAN_SECOND_HALF_START = 22;

int songPosition;
bool currentTone;

int pacmanMelody[] = {
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_C3, NOTE_C4, NOTE_G3, NOTE_E3, NOTE_C4, NOTE_G3, NOTE_E3,
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_DS4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_B4
};

int pacmanNoteDurations[] = {
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  32,32,16,32,32,16,32,32,16,16,8
};

int marioMelody[] = {
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_DS4, NOTE_C5, 0, NOTE_C5, NOTE_DS4, NOTE_C5, NOTE_DS4, NOTE_C5,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C3, NOTE_B4, 0, NOTE_B4, NOTE_C3, NOTE_B4, NOTE_C3, NOTE_B4
};

int marioNoteDurations[] = {
  8,8,8,16,16,16,8,16,16,16,8,
  8,8,8,16,16,16,8,16,16,16,8
};

int playPacman()
{
  noTone(PIN_TONE);
  songPosition = songPosition%(PACMAN_NOTES+1);
  int noteDuration = 1000 / pacmanNoteDurations[songPosition] * 1.5;
  tone(PIN_TONE, pacmanMelody[songPosition], noteDuration);
  int pauseBetweenNotes = noteDuration * 1.30;
  songPosition++;
  return pauseBetweenNotes;
}

int playMario()
{
  noTone(PIN_TONE);
  songPosition = songPosition%23;
  int noteDuration = 1000 / marioNoteDurations[songPosition];
  tone(PIN_TONE, marioMelody[songPosition], noteDuration);
  int pauseBetweenNotes = noteDuration * 1.30;
  songPosition++;
  return pauseBetweenNotes;
}