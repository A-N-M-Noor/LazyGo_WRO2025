#ifndef IO_PINS_H
#define IO_PINS_H

#include <Arduino.h>

// Pin map
#define BUTTON_PIN 5
#define STATUS_LED 4
#define BUZZER_PIN 2
#define IR1_PIN 36
#define IR2_PIN 39
#define IR3_PIN 34
#define IR4_PIN 35

// Buzzer tones
#define BUZZ_LOW 1000
#define BUZZ_MID 2000
#define BUZZ_HIGH 4000

// Initializes GPIO directions and does basic startup indications
void initIO();

#endif  // IO_PINS_H
