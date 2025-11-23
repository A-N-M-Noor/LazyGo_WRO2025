#ifndef IO_PINS_H
#define IO_PINS_H

#include <Arduino.h>

// Pin map
#define BUTTON_PIN 5
#define STATUS_LED 2
#define BUZZER_PIN 16
#define IR_EN 32
#define IR1_PIN 36
#define IR2_PIN 39
#define IR3_PIN 34
#define IR4_PIN 35

// Buzzer tones
#define BUZZ_LOW 264
#define BUZZ_MID 464
#define BUZZ_HIGH 824

// Initializes GPIO directions and does basic startup indications
void initIO();

#endif  // IO_PINS_H
