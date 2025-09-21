#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "MPU.h"

// MPU6050 variables
MPU6050 mpu;
#define INTERRUPT_PIN 19
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
volatile float ypr[3] = {0.0, 0.0, 0.0};
volatile float continuousYPR[3] = {0.0, 0.0, 0.0};
volatile float pollingRate = 0.0;
// volatile unsigned long startTime = 0;

volatile bool mpuInterrupt = false;

// Interrupt handler
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU() {
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(132);
    mpu.setYGyroOffset(-6);
    mpu.setZGyroOffset(26);
    mpu.setZAccelOffset(3261);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void mpuTask(void *pvParameters) {
    unsigned long lastPacketTime = 0;
    float prevYPR[3] = {0.0, 0.0, 0.0}; // Track previous angles for continuity
    while (1) {
        if (!dmpReady || !mpuInterrupt) {
            vTaskDelay(1); // Yield to other tasks
            continue;
        }
        mpuInterrupt = false;
        unsigned long mpuStart = micros();
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll((float*)ypr, &q, &gravity);

            // Update continuous angles
            for (int i = 0; i < 3; i++) {
                float currentDeg = ypr[i] * 180 / M_PI;
                float prevDeg = prevYPR[i];
                float delta = currentDeg - prevDeg;
                // Detect 360° crossing (assuming max rotation speed < 180° per 10 ms)
                if (delta > 180.0) delta -= 360.0;
                else if (delta < -180.0) delta += 360.0;
                continuousYPR[i] += delta;
                prevYPR[i] = currentDeg;
            }

            unsigned long currentTime = micros();
            if (lastPacketTime > 0) {
                float deltaTime = (float)(currentTime - lastPacketTime) / 1000000.0;
                pollingRate = 1.0 / deltaTime;
            }
            lastPacketTime = currentTime;

            // Print selected axis
            // const char* axisNames[3] = {"yaw", "pitch", "roll"};
            // Serial.print(axisNames[AXIS_IN_USE]);
            // Serial.print("\t");
            // Serial.print(continuousYPR[AXIS_IN_USE], 2);
            // Serial.print("\t polling rate: ");
            // Serial.print(pollingRate, 0);
            // Serial.print(" Hz\tMPU Time: ");
            // Serial.println(micros() - mpuStart);
        }
        vTaskDelay(1); // Yield after processing
    }
}

void startMPUTask() {
    xTaskCreatePinnedToCore(mpuTask, "MPUTask", 4096, NULL, 10, NULL, 0); // Core 0, high priority
}