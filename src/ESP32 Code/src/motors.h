#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

extern const float TPM;  // Ticks per meter

#define SERVO_PIN 18
#define CAM_SERVO_PIN 17
#define CAM_SERVO_MIN_US 500
#define CAM_SERVO_MAX_US 2340
#define CAM_SERVO_CENTER_US 1420
#define CAM_SERVO_ANGLE_MIN 19
#define CAM_SERVO_ANGLE_MAX 161

#define SERVO_MIN_US 950      // steers robot to the right
#define SERVO_MAX_US 2250     // steers robot to the left
#define SERVO_CENTER_US 1600  // center position for servo (Adjust if needed)

#define MOTOR_IN1 27
#define MOTOR_IN2 26
#define MOTOR_PWM 25
#define MOTOR_ENB 14

#define ENCODER_A 13
#define ENCODER_B 33

// Legacy wheel/encoder constants removed; we use TPM (ticks per meter)
// #define MOTOR_PWM_FREQ 18000
// #define MOTOR_PWM_CHANNEL 0

class Motors {
   private:
    // Servo servo;
    volatile long encoderCount;
    float target_speed_mms;     // Target speed for PI control (mm/s)
    bool do_control;            // Enable/disable speed control
    static Motors* instance;    // Static pointer to the Motors instance
    static void IRAM_ATTR encoderISR();
    static void speedControlTask(void* pvParameters);  // FreeRTOS task for PID control

   public:
    void begin();
    void setServoUs(uint32_t pulse_width_us);  // Set servo pulse width in microseconds
    void setServoAngle(uint8_t angle0to100);   // Set servo angle from 0 to 100 (0% to 100%)
    void setCamServoUs(uint32_t pulse_width_us);
    void setMotorSpeed(float speed_mms);   // Set motor speed in mm/s (positive forward, negative reverse)
    void stop();                           // Stop the motors (speed target = 0)
    void hardBreak();                      // Apply a hard brake to the motors
    long getEncoderCount();                // Get the current encoder count
    void run(int spd_vlu);
    void control_enabled(bool en);
};

#endif