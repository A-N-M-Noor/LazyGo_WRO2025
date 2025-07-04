#include <Arduino.h>
#include "MPU.h"
#include "OLED.h"
#include "battery.h"




void setup() {
    Serial.begin(115200);
    while (!Serial);

    initADC();    // Initialize battery ADC
    initMPU();    // Initialize MPU6050
    initOLED();   // Initialize OLED
    startTime = millis(); // Set start time

    // Start FreeRTOS tasks
    startMPUTask();        // MPU task on Core 0
    startOLEDDisplayTask(); // OLED task on Core 1
}

void loop() {
    // Empty loop; tasks handle everything
    
    vTaskDelay(100 / portTICK_PERIOD_MS); // Yield to other tasks
}