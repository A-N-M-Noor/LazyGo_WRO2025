#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

#define AXIS_IN_USE 0 // 0 for yaw, 1 for pitch, 2 for roll

// Shared variables for MPU data
extern volatile float ypr[3]; // Yaw, pitch, roll in radians
extern volatile float continuousYPR[3]; // Continuous angles in degrees
extern volatile float pollingRate; // Polling rate in Hz
extern volatile unsigned long startTime; // System start time in ms

// Function declarations
void initMPU();
void startMPUTask();

#endif // MPU_H