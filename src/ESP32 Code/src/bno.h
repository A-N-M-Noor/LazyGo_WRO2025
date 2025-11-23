#ifndef BNO_H
#define BNO_H

#include <Arduino.h>
// unsigned long startTime = 0;
// I2C pin definitions (shared)
#define I2C_SDA 21
#define I2C_SCL 22

// Shared variables (safe to read from other tasks)
extern volatile float heading;           // Heading (deg), offset-corrected
extern volatile float heading_norm;      // Heading - Normalized between 0 to 360
extern volatile float offs;              // Heading offset (deg)
extern volatile float positionXY[2];     // x, y position estimate
extern volatile float headingVel;        // Speed along heading
extern volatile float pollingRateBNO;    // Sampling rate (Hz)
extern volatile unsigned long startTime; // System start time in ms

// Function declarations
void initBNO();
void startBNOTask();
void startBNOCalcTask();
void bnoCalc();
void bnoCalcOffset(int dur);
void bnoCalcTask(void* pvParameters);
void bnoResetNormalize();
// Optional: low-level polling task (created by startBNOTask)
void bnoTask(void* pvParameters);
double rad(float deg);

#endif // BNO_H
