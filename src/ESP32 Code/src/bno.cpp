#include "bno.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

// Sensor instance: Address 0x29 is common for Adafruit breakouts
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

// --- Shared Variables (Volatile for ISR/Task safety) ---
volatile float _heading = 0.0;              // Continuous heading (can go beyond 360)
volatile float heading = 0.0;               // Heading relative to offset
volatile float heading_norm = 0.0;          // Normalized heading (-180 to 180)
volatile float positionXY[2] = {0.0, 0.0};  // Estimated X, Y (Dead reckoning)
volatile float headingVel = 0.0;            // Angular velocity
volatile float pollingRateBNO = 0.0;        // Debug: Actual sensor read rate
volatile unsigned long startTime = 0;

// Physics constants for integration
static uint16_t sampleDelay = 10;  // ms
static double accelVelTransition = (double)sampleDelay / 1000.0;
static double accelPosTransition = 0.5 * accelVelTransition * accelVelTransition;
static double deg2rad = 0.01745329251;

uint16_t printCount = 0;
uint32_t lastSampleMicros = 0;

float prevA = 0;        // Previous angle for delta calculation
volatile float offs = 0; // Zero-heading offset

void initBNO() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // Attempt to connect to sensor
    if (!bno.begin()) {
        while (1)
            delay(10); // Halt if sensor not found
    }

    bno.setExtCrystalUse(true); // Use external crystal for better accuracy

    // IMUPLUS mode: Uses Accel + Gyro (Fusion), but ignores Magnetometer
    // This is often more stable indoors where magnetic interference is high.
    bno.setMode(OPERATION_MODE_IMUPLUS);
    delay(100);

    // --- Hardcoded Calibration Offsets ---
    // These values should be obtained by running a calibration sketch once
    // and saving the values here to avoid calibrating on every boot.
    adafruit_bno055_offsets_t calibData;
    calibData.accel_offset_x = -27;
    calibData.accel_offset_y = 16;
    calibData.accel_offset_z = 5;
    calibData.gyro_offset_x = -1;
    calibData.gyro_offset_y = 1;
    calibData.gyro_offset_z = 1;

    calibData.mag_offset_x = 769;
    calibData.mag_offset_y = 616;
    calibData.mag_offset_z = -830;

    calibData.accel_radius = 1000;
    calibData.mag_radius = 1211;

    bno.setSensorOffsets(calibData);

    delay(1000); // Allow sensor to settle
}

/**
 * FreeRTOS Task: Reads raw sensor data.
 * Attempts to integrate acceleration for position (experimental/drifty).
 */
void bnoTask(void* pvParameters) {
    sensors_event_t orientationData, linearAccelData;
    unsigned long lastPacketTime = 0;

    while (1) {
        unsigned long tStart = micros();

        // Get Orientation (Euler angles) and Linear Acceleration (Gravity removed)
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // Update global heading
        heading = orientationData.orientation.x;

        // Double integration for position (Accel -> Vel -> Pos)
        // Note: This is highly prone to drift on consumer IMUs.
        positionXY[0] += accelPosTransition * linearAccelData.acceleration.x;
        positionXY[1] += accelPosTransition * linearAccelData.acceleration.y;

        // Calculate angular velocity
        headingVel = accelVelTransition * linearAccelData.acceleration.x /
                     cos(deg2rad * heading);

        // Calculate polling rate for debug
        unsigned long now = micros();
        if (lastPacketTime > 0) {
            float deltaTime = (float)(now - lastPacketTime) / 1000000.0;
            pollingRateBNO = 1.0 / deltaTime;
        }
        lastPacketTime = now;

        // Reset print counter
        if (printCount * sampleDelay >= 500) {
            printCount = 0;
        } else {
            printCount++;
        }

        // Busy-wait to maintain precise sampling interval
        while ((micros() - tStart) < (sampleDelay * 1000)) {
            // wait
        }
        vTaskDelay(1);  // Yield to other tasks
    }
}

/**
 * Helper: Calculates continuous heading.
 * The BNO outputs 0-360. This function tracks wraps to allow
 * headings like 370, 720, etc., or negative values.
 */
void bnoCalc() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float a = euler.x();
    
    // Calculate delta change
    float dA = a - prevA;
    
    // Handle wrap-around (e.g., 359 -> 1)
    if (dA > 180)
        dA -= 360;
    else if (dA < -180)
        dA += 360;

    _heading += dA; // Accumulate total rotation
    heading = _heading - offs; // Apply zero offset
    
    // Calculate normalized heading (-180 to 180)
    heading_norm = fmod(heading, 360);

    if (heading_norm < 0) {
        heading_norm += 360;
    }
    if(heading_norm > 180){
        heading_norm -= 360;
    }
    prevA = a;
}

void bnoResetNormalize(){
    heading = heading_norm;
    if(heading > 180){
        heading -= 360;
    }
}

/**
 * FreeRTOS Task: Runs the heading calculation loop.
 */
void bnoCalcTask(void* pvParameters) {
    while(1) {
        bnoCalc();
        vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
    }
}

/**
 * Sets the current heading as the new "Zero".
 * Useful for resetting orientation at the start of a run.
 */
void bnoCalcOffset(int dur) {
    long _start = millis();
    // Wait for a duration to ensure stable reading
    vTaskDelay(dur / portTICK_PERIOD_MS);   
    offs = _heading; // Store current accumulated heading as offset
}

double rad(float deg) {
    return deg * deg2rad;
}

// --- Task Launchers ---

void startBNOTask() {
    xTaskCreatePinnedToCore(
        bnoTask,
        "BNOTask",
        4096,
        NULL,
        5,
        NULL,
        0
    );
}

void startBNOCalcTask() {
    xTaskCreatePinnedToCore(
        bnoCalcTask,
        "BNOCalcTask",
        4096,
        NULL,
        5,
        NULL,
        0  // Core 0
    );
}
