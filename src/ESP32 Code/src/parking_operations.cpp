#include "parking_operations.h"
#include "motors.h"
#include "OLED.h"
#include "battery.h"
#include "MPU.h"
#include "bno.h"

// Remove this problematic line:
// #include "main.cpp"

// Declare external variables from main.cpp
extern Motors motors;
extern float TPM; // Ticks per meter
// extern float heading; // Removed to fix conflicting declaration error
int parking_pwm = 60;

// Declare external functions from main.cpp
extern void bnoCalc();
extern void odometry();
extern void srl();


void turn_angle(float target_angle) {
    bnoCalc();
    odometry();
    float error = target_angle - heading;

    if(error > 0) {
        motors.setServoUs(SERVO_MIN_US); // Turn right
    } else {
        motors.setServoUs(SERVO_MAX_US); // Turn left
    }

    delay(100);

    while (abs(error) > 2) { // Allowable error margin
        bnoCalc();
        odometry();
        // srl();

        error = target_angle - heading;
        motors.run(parking_pwm);
        if(error > 0) {
            motors.setServoUs(SERVO_MIN_US); // Turn right
        } else {
            motors.setServoUs(SERVO_MAX_US); // Turn left
        }

        delay(10);
    }
    motors.run(-parking_pwm);
    delay(80);
    motors.run(0);
}

void move_pos(float distance){
    int tick_now = motors.getEncoderCount();
    int tick_target = tick_now + int(distance * TPM);
    
    bnoCalc();
    odometry();
    // srl();

    if(tick_target > tick_now) {
        motors.run(parking_pwm); // Forward
    } else {
        motors.run(-parking_pwm); // Backward
    }

    while (abs(tick_target - motors.getEncoderCount()) > 10) {
        bnoCalc();
        odometry();
        // srl();
        delay(20);
    }

    motors.run(-parking_pwm);
    delay(80);
    motors.run(0);
}