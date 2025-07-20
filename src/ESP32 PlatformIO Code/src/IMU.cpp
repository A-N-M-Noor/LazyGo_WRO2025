#include <Arduino.h>
#include "IMU.h"

// Global variables
        float rawYaw;
float continuousYaw = 0.0; // Stores the continuous yaw angle (degrees)
bool IMUerror = false; // True if IMUTask exceeds BNO055_SAMPLERATE_DELAY_MS
float pollingRate = 0.0; // Polling frequency of IMUTask (Hz)
uint32_t startTime = 0; // Stores millis() when IMU is initialized
bool isYawTracking = false; // True when continuous yaw tracking starts (after 10s)
static float lastRawYaw = 0.0; // Stores the last raw yaw reading for wrap-around detection
static float yawAdjustment = 0.0; // Cumulative adjustment for continuous yaw
static float referenceYaw = 0.0; // Yaw value when tracking starts (set to 0)

// I2C instance for BNO055
TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);

/**
 * Initializes the BNO055 sensor
 */
void initIMU(void) {
    startTime = millis(); // Record initialization time
    I2CBNO.begin(I2C_SDA, I2C_SCL);
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for serial port

    Serial.println("Orientation Sensor Test");
    Serial.println("");

    if (!bno.begin()) {
        Serial.print("No BNO055 detected ... Check wiring or I2C address!");
        while (1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    // Optional: Display sensor details
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
}

/**
 * Task to periodically read BNO055 and update continuous yaw
 */
void IMUTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t lastTime = micros();
    while (1) {
        uint32_t tStart = micros();
        
        // Get orientation data
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

        // Get raw yaw based on AXIS_IN_USE
        switch (AXIS_IN_USE) {
            case 0: rawYaw = orientationData.orientation.x; break;
            case 1: rawYaw = orientationData.orientation.y; break;
            case 2: rawYaw = orientationData.orientation.z; break;
        }

        // Check if 10 seconds have passed to start yaw tracking
        if (!isYawTracking && (millis() - startTime >= 10000)) {
            resetYawTracking();
        }

        // Update continuous yaw only if tracking is enabled
        if (isYawTracking) {
            // Handle yaw wrap-around for continuity
            float deltaYaw = rawYaw - lastRawYaw;
            if (deltaYaw > 180.0) {
                yawAdjustment -= 360.0; // Negative wrap-around (e.g., 1 -> 359)
            } else if (deltaYaw < -180.0) {
                yawAdjustment += 360.0; // Positive wrap-around (e.g., 359 -> 1)
            }
            continuousYaw = (rawYaw - referenceYaw) + yawAdjustment;
            lastRawYaw = rawYaw;
        }

        // Calculate polling rate (Hz) based on time since last execution
        uint32_t currentTime = micros();
        uint32_t deltaTime = currentTime - lastTime;
        pollingRate = (deltaTime > 0) ? 1000000.0 / deltaTime : 0.0;
        lastTime = currentTime;

        // Check if task execution time exceeds BNO055_SAMPLERATE_DELAY_MS
        uint32_t executionTime = (micros() - tStart) / 1000;
        if (executionTime > BNO055_SAMPLERATE_DELAY_MS) {
            IMUerror = true;
            Serial.print("IMU Task overrun: ");
            Serial.print(executionTime);
            Serial.println(" ms");
        } else {
            IMUerror = false;
        }

        // Delay until next fixed interval (10ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO055_SAMPLERATE_DELAY_MS));
    }
}

/**
 * Starts the IMU task using FreeRTOS
 */
void startIMUTask(void) {
    xTaskCreate(
        IMUTask,           // Task function
        "IMUTask",         // Task name
        2048,              // Stack size (bytes)
        NULL,              // Task parameters
        1,                 // Priority
        NULL               // Task handle
    );
}

void resetYawTracking() {
    isYawTracking = true;
    referenceYaw = rawYaw; // Set the current raw yaw as the reference
    lastRawYaw = rawYaw; // Initialize lastRawYaw
    yawAdjustment = 0.0; // Reset cumulative adjustment
    continuousYaw = 0.0; // Reset continuous yaw
    Serial.println("Continuous yaw tracking started.");
}