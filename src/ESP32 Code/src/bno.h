#ifndef BNO_H
#define BNO_H

#include <Arduino.h>
// unsigned long startTime = 0;
// I2C pin definitions (shared)
#define I2C_SDA 21
#define I2C_SCL 22

// Shared variables (safe to read from other tasks)
extern volatile float heading;          // heading (deg)
extern volatile float offs;         // uncorrected heading (deg)
extern volatile float positionXY[2];    // x, y position estimate
extern volatile float headingVel;       // speed along heading
extern volatile float pollingRateBNO;   // Hz
extern volatile unsigned long startTime; // System start time in ms

// Function declarations
void initBNO();
void startBNOTask();
void bnoCalc();
void bnoCalcOffset(int dur);
double rad(float deg);

#endif // BNO_H
