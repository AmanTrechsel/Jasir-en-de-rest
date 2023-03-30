// ################################################################################################################################
// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Bot Gamma *
//                                                  Melody Handler
// --------------------------------------------------------------------------------------------------------------------------------
//                                                 Jasir en de rest
//                                                   Version: 3.4
// --------------------------------------------------------------------------------------------------------------------------------
//                                                    Written by:
//                                                   Aman Trechsel
//                                                  Jasir Abdikarim
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
// ################################################################################################################################

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Libraries *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
#include "Notes.h"                                   // For notes

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                  * Constants *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
const int PIN_TONE = 8;                              // Pin at which the toner is connected
const int PACMAN_NOTES = 32;                         // Amount of notes of the Pac-man song
const int PACMAN_SECOND_HALF_START = 22;             // Amount of notes before the second half of the Pac-man song


// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                   * Variables *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
int songPosition;                                    // Position of the playing song

int pacmanMelody[] = {                               // All notes in the Pac-man song
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_C3, NOTE_C4, NOTE_G3, NOTE_E3, NOTE_C4, NOTE_G3, NOTE_E3,
  NOTE_B3, NOTE_B4, NOTE_FS3, NOTE_DS3, NOTE_B4, NOTE_FS3, NOTE_DS3,
  NOTE_DS4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_B4
};

int pacmanNoteDurations[] = {                        // All durations for the notes in the Pac-man song
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  16,16,16,16,32,12,8,
  32,32,16,32,32,16,32,32,16,16,8
};

int marioMelody[] = {                                // All notes in the Mario song
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_DS4, NOTE_C5, 0, NOTE_C5, NOTE_DS4, NOTE_C5, NOTE_DS4, NOTE_C5,
  NOTE_B4, NOTE_B4, NOTE_B4, NOTE_C3, NOTE_B4, 0, NOTE_B4, NOTE_C3, NOTE_B4, NOTE_C3, NOTE_B4
};

int marioNoteDurations[] = {                         // All durations for the notes in the Mario song
  8,8,8,16,16,16,8,16,16,16,8,
  8,8,8,16,16,16,8,16,16,16,8
};

// ================================================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------
//                                                      * Methods *
// --------------------------------------------------------------------------------------------------------------------------------
// ================================================================================================================================
int playPacman()
{
  // Stop if already playing something
  noTone(PIN_TONE);
  // Ensure the position of the song is within the boundaries
  songPosition = songPosition%(PACMAN_NOTES+1);
  // Calculate the duration of this note
  int noteDuration = 1000 / pacmanNoteDurations[songPosition] * 1.5;
  // Play the note
  tone(PIN_TONE, pacmanMelody[songPosition], noteDuration);
  // Increment the current position of the song
  songPosition++;
  // Calculate how long until the next note is played
  int pauseBetweenNotes = noteDuration * 1.30;
  // Return the wait time
  return pauseBetweenNotes;
}

int playMario()
{
  // Stop if already playing something
  noTone(PIN_TONE);
  // Ensure the position of the song is within the boundaries
  songPosition = songPosition%23;
  // Calculate the duration of this note
  int noteDuration = 1000 / marioNoteDurations[songPosition];
  // Play the note
  tone(PIN_TONE, marioMelody[songPosition], noteDuration);
  // Increment the current position of the song
  songPosition++;
  // Calculate how long until the next note is played
  int pauseBetweenNotes = noteDuration * 1.30;
  // Return the wait time
  return pauseBetweenNotes;
}