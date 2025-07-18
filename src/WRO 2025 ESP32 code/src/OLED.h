#ifndef OLED_H
#define OLED_H

#include <Arduino.h>

#define DISPLAY_UPDATE_INTERVAL 1000 // 1000 ms (1 Hz)

// Function declarations
void initOLED();
void startOLEDDisplayTask();

#endif // OLED_H