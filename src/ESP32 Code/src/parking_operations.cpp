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
    float initErr = error;

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
    // if(abs(initErr) > 30){
        motors.run(-parking_pwm);
        delay(80);
    // }
    motors.run(0);
}

void turn_angle_opp(float target_angle) {
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
        motors.run(-parking_pwm);
        if(error < 0) {
            motors.setServoUs(SERVO_MIN_US); // Turn right
        } else {
            motors.setServoUs(SERVO_MAX_US); // Turn left
        }

        delay(10);
    }
    motors.run(parking_pwm);
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

void head_into(){
    odometry();
    bnoCalc();
    int tickNow = motors.getEncoderCount();
    float trg = 0;
    if(heading > 0){
        trg = 360*3;
    }
    else{
        trg = -360*3;
    }

    motors.run(parking_pwm);
    long tmr = millis();

    while(true){
        odometry();
        bnoCalc();
        int sv = map(trg - heading, -45, 45, SERVO_MAX_US, SERVO_MIN_US);
        motors.setServoUs(sv);
        delay(20);

        if(millis() - tmr > 1000){
            if(motors.getEncoderCount() != tickNow){
                tickNow = motors.getEncoderCount();
            }
            else{
                break;
            }
        }
    }
}