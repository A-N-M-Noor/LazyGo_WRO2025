#include "parking_operations.h"

#include "OLED.h"
#include "battery.h"
#include "bno.h"
#include "motors.h"
#include "io_pins.h"

extern Motors motors;
int parking_speed = 120; // Default speed for parking moves
int parking_pwm = 60;    // Low speed PWM

float turn_err_threshold = 2.0;  // Acceptable error in degrees for turns

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

// Checks if either of the specific IR sensors detects a wall
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

/**
 * Turns the robot to a specific absolute heading.
 * Uses a simple P-controller logic: turns wheels until error < threshold.
 */
void turn_angle(float target_angle) {
    float error = target_angle - heading_norm;
    float initErr = error;

    // Initial steering set
    if (error > 0) {
        motors.setServoUs(SERVO_MIN_US);  // Turn right
    } else {
        motors.setServoUs(SERVO_MAX_US);  // Turn left
    }

    delay(100); // Wait for servo to move

    // Loop until target reached or IR sensor triggers
    while (abs(error) > turn_err_threshold) {
        error = target_angle - heading_norm;
        motors.setMotorSpeed(parking_speed); // Move forward
        
        // Adjust steering based on error direction
        if (error > 0) {
            motors.setServoUs(SERVO_MIN_US);  // Turn right
        } else {
            motors.setServoUs(SERVO_MAX_US);  // Turn left
        }

        delay(10);
        if(IR_any()){
            break; // Stop if wall detected
        }
    }
    
    // Braking maneuver: briefly reverse
    motors.setServoUs(SERVO_CENTER_US);
    motors.setMotorSpeed(-parking_speed);
    delay(80);
    motors.setMotorSpeed(0);
}

/**
 * Turns the robot in reverse (Opposite direction).
 * Useful for backing into spots.
 */
void turn_angle_opp(float target_angle) {
    float error = target_angle - heading_norm;

    // Initial steering
    if (error > 0) {
        motors.setServoUs(SERVO_MIN_US);  // Turn right
    } else {
        motors.setServoUs(SERVO_MAX_US);  // Turn left
    }

    delay(100);

    while (abs(error) > turn_err_threshold) {
        error = target_angle - heading_norm;
        motors.setMotorSpeed(-parking_speed); // Move Backward
        
        // Steering logic is inverted for reverse 
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
    // Braking: briefly move forward
    motors.setMotorSpeed(parking_speed);
    delay(80);
    motors.setMotorSpeed(0);
}

/**
 * Moves the robot a specific distance while maintaining a target heading.
 * Uses encoders for distance and IMU for heading correction.
 */
void move_pos_func(float distance, float trg_ang) {
    // "Kick" start for small distances to overcome static friction
    if(distance > 0 && distance < 0.05){
        motors.control_enabled(false);
        motors.run(255);
        delay(50);
        motors.run(0);
        motors.control_enabled(true);
    }
    int dir = 1;

    // Calculate target encoder ticks
    int tick_now = motors.getEncoderCount();
    int tick_target = tick_now + int(distance * TPM); // TPM = Ticks Per Meter

    if (tick_target > tick_now) {
        motors.setMotorSpeed(parking_speed);  // Forward
    } else {
        motors.setMotorSpeed(-parking_speed);  // Backward
        dir = -1;
    }

    // Move until encoder target reached
    while (abs(tick_target - tick_now) > int(0.005 * TPM)) {
        tick_now = motors.getEncoderCount();
        
        // Correction if we overshoot or oscillate
        if (tick_target > tick_now && dir == -1) {
            motors.setMotorSpeed(parking_speed);  // Forward
            dir = 1;
        } else if (tick_target < tick_now && dir == 1) {
            motors.setMotorSpeed(-parking_speed);  // Backward
            dir = -1;
        }
        
        // Proportional steering correction to maintain straight line
        // Maps heading error (-20 to 20 deg) to Servo range
        int sv = map(trg_ang - heading_norm, -20*dir, 20*dir, SERVO_MAX_US, SERVO_MIN_US);
        motors.setServoUs(sv);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Brake
    motors.setMotorSpeed(-parking_speed);
    delay(80);
    motors.setMotorSpeed(0);
}

// Overloads for move_pos
void move_pos(float distance) {
    float init_heading = heading_norm;
    move_pos_func(distance, init_heading); // Maintain current heading
}
void move_pos(float distance, float target_heading) {
    move_pos_func(distance, target_heading); // Maintain specific heading
}

/**
 * Special maneuver: Backs up while steering to align with a target.
 * Stops based on time or distance traveled.
 */
void head_into(float trg) {
    int tickNow = motors.getEncoderCount();

    motors.control_enabled(false);
    motors.run(-parking_pwm); // Slow reverse
    long tmr = millis();
    long travelStart = tickNow;

    while (true) {
        // Steer based on heading error
        int sv = map(trg - heading_norm, 45, -45, SERVO_MAX_US, SERVO_MIN_US);
        motors.setServoUs(sv);
        delay(20);

        // Safety timeout: Check if encoder is moving
        if (millis() - tmr > 1000) {
            if (motors.getEncoderCount() != tickNow) {
                tickNow = motors.getEncoderCount();
            } else {
                break; // Stalled
            }
        }
        // Stop after traveling 1 meter
        if(abs(motors.getEncoderCount() - travelStart) > TPM * 1.0){
            break;
        }
    }
    motors.run(0);
    motors.setMotorSpeed(0);
    motors.control_enabled(true);
}