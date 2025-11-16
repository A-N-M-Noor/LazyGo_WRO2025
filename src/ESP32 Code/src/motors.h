#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#define SERVO_PIN 18
#define CAM_SERVO_PIN 17
#define CAM_SERVO_MIN_US 500
#define CAM_SERVO_MAX_US 2400
#define CAM_SERVO_CENTER_US 1450

#define SERVO_MIN_US 600      // steers robot to the right
#define SERVO_MAX_US 2000     // steers robot to the left
#define SERVO_CENTER_US 1300  // center position for servo (Adjust if needed)1125

#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define MOTOR_PWM 25
#define MOTOR_ENB 14

#define ENCODER_A 12
#define ENCODER_B 13

#define WHEEL_DIAMETER_CM 4.829
#define PULSES_PER_REVOLUTION 400  // Tune this to get correct distance movement
#define MOTOR_PWM_FREQ 18000       // 18kHz PWM frequency to avoid noise
#define MOTOR_PWM_CHANNEL 0

class Motors {
   private:
    // Servo servo;
    volatile long encoderCount;
    float target_speed_mms;     // Target speed for PID control
    float target_position_mm;   // Target position in millimeters
    float current_position_mm;  // Current position in millimeters
    bool use_position_control;  // Flag to select position or speed control
    static Motors* instance;    // Static pointer to the Motors instance
    static void IRAM_ATTR encoderISR();
    static void speedControlTask(void* pvParameters);  // FreeRTOS task for PID control

   public:
    void begin();
    void setServoUs(uint32_t pulse_width_us);  // Set servo pulse width in microseconds
    void setServoAngle(uint8_t angle0to100);   // Set servo angle from 0 to 100 (0% to 100%)
    void setCamServoUs(uint32_t pulse_width_us);
    void setMotorSpeed(float speed_mms);           // Set motor speed in mm/s (positive for forward, negative for backward)
    void moveDistance(float cm, float speed_mms);  // Move a distance in centimeters at a specified speed in mm/s
    void stop();                                   // Stop the motors
    void hardBreak();                              // Apply a hard brake to the motors
    long getEncoderCount();                        // Get the current encoder count
    float getCurrentPosition();                    // Get the current position in millimeters
    void run(int spd_vlu);
};

#endif