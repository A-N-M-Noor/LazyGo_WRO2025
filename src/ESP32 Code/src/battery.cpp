#include <Arduino.h>
#include "battery.h"

// Pin and ADC configuration
#define ADC_PIN 15 // GPIO27 (ADC2_CHANNEL_0)
#define SAMPLES 16 // Number of samples for averaging
#define VREF 1100.0 // Default reference voltage in mV
#define VOLTAGE_MULTIPLIER 10.6727272727 // Multiplier to scale battery voltage
#define LOW_VOLTAGE_LIMIT 11 // Low voltage threshold in volts
#define UPDATE_INTERVAL 5000 // 60 seconds in milliseconds

// Global variables
bool batteryLow = false;
double batteryVoltage = 0.0;

// Static variables for timing
static unsigned long lastUpdateTime = 0;

void initADC() {
  // Configure ADC resolution
  analogReadResolution(12); // 12-bit resolution (0–4095)
  // Set attenuation for ADC2 (2.5 dB, ~0–1.5V range)
  analogSetPinAttenuation(ADC_PIN, ADC_2_5db);
}

double readBatteryVoltage() {
  uint32_t adcReading = 0;
  
  // Multisampling for noise reduction
  for (int i = 0; i < SAMPLES; i++) {
    adcReading += analogRead(ADC_PIN);
    delay(10); // Small delay between samples
  }
  adcReading /= SAMPLES; // Average the readings

  // Convert raw ADC to voltage (mV), then to volts
  double voltage = (adcReading * VREF / 4095.0) / 1000.0; // Base voltage in volts
  voltage *= VOLTAGE_MULTIPLIER; // Scale by multiplier

  // Update batteryLow flag
  batteryLow = (voltage < LOW_VOLTAGE_LIMIT);

  return voltage;
}

void updateBatteryVoltage() {

  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
          // Serial.println("\nbattery update\n");
    batteryVoltage = readBatteryVoltage();
    lastUpdateTime = currentTime;
  }
}

double getBatteryVoltage() {
  return batteryVoltage;
}