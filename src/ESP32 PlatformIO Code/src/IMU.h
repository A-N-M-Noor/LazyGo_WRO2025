#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Wire.h>

// I2C pin definitions
#define I2C_SDA 21
#define I2C_SCL 22

// Sampling period for BNO055 (ms)
#define BNO055_SAMPLERATE_DELAY_MS 10

// Define which axis to use for yaw (0=X, 1=Y, 2=Z)
#ifndef AXIS_IN_USE
#define AXIS_IN_USE 0 // Default to X axis (0) for yaw
#endif

// Validate AXIS_IN_USE
#if AXIS_IN_USE != 0 && AXIS_IN_USE != 1 && AXIS_IN_USE != 2
#error "AXIS_IN_USE must be 0 (X), 1 (Y), or 2 (Z)"
#endif

// Global variables
extern float rawYaw; // Stores the raw yaw angle (degrees) from the selected axis
extern float continuousYaw; // Stores the continuous yaw angle (degrees) from the selected axis
extern bool IMUerror; // True if IMUTask execution time exceeds BNO055_SAMPLERATE_DELAY_MS
extern float pollingRate; // Polling frequency of IMUTask (Hz)
extern uint32_t startTime; // Stores millis() when IMU is initialized
extern bool isYawTracking; // True when continuous yaw tracking starts (after 10s)

// Function prototypes
void initIMU(void);
void startIMUTask(void);
void IMUTask(void *pvParameters);
void resetYawTracking(); // Resets yaw tracking state and initializes reference yaw

#endif // IMU_H