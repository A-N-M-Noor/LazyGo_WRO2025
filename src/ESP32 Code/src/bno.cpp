#include "bno.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <utility/imumaths.h>

// Sensor instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);  // change 0x28 <-> 0x29 if needed

// Shared variables
volatile float _heading = 0.0;
volatile float heading = 0.0;
volatile float heading_norm = 0.0;
volatile float positionXY[2] = {0.0, 0.0};  // x, y
volatile float headingVel = 0.0;
volatile float pollingRateBNO = 0.0;
volatile unsigned long startTime = 0;

static uint16_t sampleDelay = 10;  // ms
static double accelVelTransition = (double)sampleDelay / 1000.0;
static double accelPosTransition = 0.5 * accelVelTransition * accelVelTransition;
static double deg2rad = 0.01745329251;

uint16_t printCount = 0;
uint32_t lastSampleMicros = 0;

float prevA = 0;
volatile float offs = 0;

void initBNO() {
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!bno.begin()) {
        while (1)
            delay(10);
    }

    bno.setExtCrystalUse(true);

    bno.setMode(OPERATION_MODE_IMUPLUS);
    delay(100);

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

    delay(1000);
}

void bnoTask(void* pvParameters) {
    sensors_event_t orientationData, linearAccelData;

    unsigned long lastPacketTime = 0;

    while (1) {
        unsigned long tStart = micros();

        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // Update shared variables
        heading = orientationData.orientation.x;

        positionXY[0] += accelPosTransition * linearAccelData.acceleration.x;
        positionXY[1] += accelPosTransition * linearAccelData.acceleration.y;

        headingVel = accelVelTransition * linearAccelData.acceleration.x /
                     cos(deg2rad * heading);

        // Update polling rate
        unsigned long now = micros();
        if (lastPacketTime > 0) {
            float deltaTime = (float)(now - lastPacketTime) / 1000000.0;
            pollingRateBNO = 1.0 / deltaTime;
        }
        lastPacketTime = now;

        // Debug print (every 500ms)
        if (printCount * sampleDelay >= 500) {
            printCount = 0;
        } else {
            printCount++;
        }

        // Keep timing consistent
        while ((micros() - tStart) < (sampleDelay * 1000)) {
            // wait
        }
        vTaskDelay(1);  // yield to other tasks
    }
}

void bnoCalc() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float a = euler.x();
    float dA = a - prevA;
    if (dA > 180)
        dA -= 360;
    else if (dA < -180)
        dA += 360;

    _heading += dA;
    heading = _heading - offs;
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

void bnoCalcTask(void* pvParameters) {
    while(1) {
        bnoCalc();
        vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
    }
}

void bnoCalcOffset(int dur) {
    long _start = millis();
    // while (millis() - _start < dur) {
    //     bnoCalc();
    //     vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
    // }
    vTaskDelay(dur / portTICK_PERIOD_MS);   
    offs = _heading;
}

double rad(float deg) {
    return deg * deg2rad;
}

void startBNOTask() {
    xTaskCreatePinnedToCore(
        bnoTask,
        "BNOTask",
        4096,
        NULL,
        5,
        NULL,
        0  // Core 0
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
