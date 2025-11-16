#include "battery.h"

#include <Arduino.h>

#include "io_pins.h"

// Pin and ADC configuration
#define ADC_PIN 4                // GPIO4 (ADC2_CHANNEL_)
#define SAMPLES 20                // Number of samples for averaging
#define VREF 2.2                  // Default reference voltage in volts
#define VOLTAGE_MULTIPLIER 10     // Calibrate this: Multiplier to scale battery voltage
#define LOW_VOLTAGE_LIMIT 10.5    // Low voltage threshold in volts
#define UPDATE_INTERVAL 2 * 1000  // 60 seconds in milliseconds

// Global variables
bool batteryLow = false;
double batteryVoltage = 0.0;  // in volts

// Static variables for timing
static unsigned long lastUpdateTime = 0;

void initADC() {
    // Configure ADC resolution
    analogReadResolution(12);  // 12-bit resolution (0–4095)
    // Set attenuation for ADC2 (6 dB, ~0–2.0V range)
    analogSetPinAttenuation(ADC_PIN, ADC_6db);
}

double readBatteryVoltage() {
    uint32_t adcReading = 0;

    // Multisampling for noise reduction
    for (int i = 0; i < SAMPLES; i++) {
        adcReading += analogRead(ADC_PIN);
        delay(10);  // Small delay between samples
    }
    adcReading /= SAMPLES;  // Average the readings

    // Convert raw ADC to voltage (V)
    double voltage = (adcReading * VREF / 4095.0);  // Base voltage in volts
    voltage *= VOLTAGE_MULTIPLIER;                  // Scale by multiplier

    // Update batteryLow flag
    batteryLow = (voltage < LOW_VOLTAGE_LIMIT);

    return voltage;
}

void updateBatteryVoltage() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        if (batteryLow) {
            tone(BUZZER_PIN, BUZZ_HIGH, 1000);  // Buzz for warning
            // delay without blocking other tasks
            unsigned long startBuzz = millis();
            while (millis() - startBuzz < 1000) {
                // just wait
                vTaskDelay(10 / portTICK_PERIOD_MS);  // yield to other tasks
            }
        }
        // Serial.println("\nbattery update\n");
        batteryVoltage = readBatteryVoltage();
        lastUpdateTime = currentTime;
    }
}

double getBatteryVoltage() {
    return batteryVoltage;
}