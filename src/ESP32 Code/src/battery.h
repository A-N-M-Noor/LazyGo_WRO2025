#ifndef BATTERY_H
#define BATTERY_H

// Function declarations
void initADC();
void updateBatteryVoltage(); // Changed to update function for periodic checks
double getBatteryVoltage();  // Getter for battery voltage

// Global variables
extern bool batteryLow;
extern double batteryVoltage;

#endif // BATTERY_H