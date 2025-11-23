#include "parking_operations.h"

#include "OLED.h"
#include "battery.h"
#include "bno.h"
#include "motors.h"
#include "io_pins.h"

extern Motors motors;
int parking_speed = 120;
int parking_pwm = 60;

float turn_err_threshold = 2.0;  // degrees

bool useIR = false;
extern int IR_THRESH_3;
extern int IR_THRESH_4;
extern float IR_VAL_3;
extern float IR_VAL_4;


void setUseIR(bool stt){
    useIR = stt;
}

void parkingSpeedHigh(){
    parking_speed = 220;
}

void parkingSpeedLow(){
    parking_speed = 120;
}

bool IR_any(){
    if(!useIR){
        return false;
    }
    if(IR_VAL_3 < IR_THRESH_3){
        return true;
    }
    if(IR_VAL_4 < IR_THRESH_4){
        return true;
    }
    return false;
}

void turn_angle(float target_angle) {
    float error = target_angle - heading_norm;
    float initErr = error;

    if (error > 0) {
        motors.setServoUs(SERVO_MIN_US);  // Turn right
    } else {
        motors.setServoUs(SERVO_MAX_US);  // Turn left
    }

    delay(100);

    while (abs(error) > turn_err_threshold) {
        error = target_angle - heading_norm;
        motors.setMotorSpeed(parking_speed);
        if (error > 0) {
            motors.setServoUs(SERVO_MIN_US);  // Turn right
        } else {
            motors.setServoUs(SERVO_MAX_US);  // Turn left
        }

        delay(10);
        if(IR_any()){
            break;
        }
    }
    // if(abs(initErr) > 30){
    motors.setServoUs(SERVO_CENTER_US);
    motors.setMotorSpeed(-parking_speed);
    delay(80);
    // }
    motors.setMotorSpeed(0);
}


void turn_angle_opp(float target_angle) {
    float error = target_angle - heading_norm;

    if (error > 0) {
        motors.setServoUs(SERVO_MIN_US);  // Turn right
    } else {
        motors.setServoUs(SERVO_MAX_US);  // Turn left
    }

    delay(100);

    while (abs(error) > turn_err_threshold) {
        error = target_angle - heading_norm;
        motors.setMotorSpeed(-parking_speed);
        if (error < 0) {
            motors.setServoUs(SERVO_MIN_US);  // Turn right
        } else {
            motors.setServoUs(SERVO_MAX_US);  // Turn left
        }

        if(IR_any()){
            break;
        }

        delay(10);
    }
    motors.setMotorSpeed(parking_speed);
    delay(80);
    motors.setMotorSpeed(0);
}

void move_pos_func(float distance, float trg_ang) {
    if(distance > 0 && distance < 0.05){
        motors.control_enabled(false);
        motors.run(255);
        delay(50);
        motors.run(0);
        motors.control_enabled(true);
    }
    int dir = 1;

    int tick_now = motors.getEncoderCount();
    int tick_target = tick_now + int(distance * TPM);

    if (tick_target > tick_now) {
        motors.setMotorSpeed(parking_speed);  // Forward
    } else {
        motors.setMotorSpeed(-parking_speed);  // Backward
        dir = -1;
    }

    while (abs(tick_target - tick_now) > int(0.005 * TPM)) {
        tick_now = motors.getEncoderCount();
        if (tick_target > tick_now && dir == -1) {
            motors.setMotorSpeed(parking_speed);  // Forward
            dir = 1;
        } else if (tick_target < tick_now && dir == 1) {
            motors.setMotorSpeed(-parking_speed);  // Backward
            dir = -1;
        }
        int sv = map(trg_ang - heading_norm, -20*dir, 20*dir, SERVO_MAX_US, SERVO_MIN_US);
        motors.setServoUs(sv);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    motors.setMotorSpeed(-parking_speed);
    delay(80);
    motors.setMotorSpeed(0);
}

void move_pos(float distance) {
    float init_heading = heading_norm;
    move_pos_func(distance, init_heading);
}
void move_pos(float distance, float target_heading) {
    move_pos_func(distance, target_heading);
}

void head_into(float trg) {
    int tickNow = motors.getEncoderCount();

    motors.control_enabled(false);
    motors.run(-parking_pwm);
    long tmr = millis();
    long travelStart = tickNow;

    while (true) {
        int sv = map(trg - heading_norm, 45, -45, SERVO_MAX_US, SERVO_MIN_US);
        motors.setServoUs(sv);
        delay(20);

        if (millis() - tmr > 1000) {
            if (motors.getEncoderCount() != tickNow) {
                tickNow = motors.getEncoderCount();
            } else {
                break;
            }
        }
        if(abs(motors.getEncoderCount() - travelStart) > TPM * 1.0){
            break;
        }
    }
    motors.run(0);
    motors.setMotorSpeed(0);
    motors.control_enabled(true);
}