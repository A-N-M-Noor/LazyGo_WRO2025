// THIS FILE USES THE ESP32SERVO LIBRARY WHICH USES LEDC FOR SERVO PWM

// #include "motors.h"
// #include <ESP32PWM.h>

// // Initialize static member
// Motors* Motors::instance = nullptr;

// void IRAM_ATTR Motors::encoderISR() {
//     if (instance) {
//         if (digitalRead(ENCODER_B) == HIGH) {
//             instance->encoderCount++;
//         } else {
//             instance->encoderCount--;
//         }
//     }
// }

// void Motors::begin() {
//     instance = this; // Set the static instance pointer
//     encoderCount = 0; // Initialize encoder count

//     // Allocate timers for ESP32Servo
//     // ESP32PWM::allocateTimer(0);
//     // ESP32PWM::allocateTimer(1);
//     // ESP32PWM::allocateTimer(2);
//     // ESP32PWM::allocateTimer(3);
    
//     // Configure servo for 50 Hz and specific pulse widths
//     servo.setPeriodHertz(50); // Standard 50 Hz servo
//     servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US); 

//     // Configure motor pins
//     pinMode(MOTOR_IN1, OUTPUT);
//     pinMode(MOTOR_IN2, OUTPUT);
//     pinMode(MOTOR_PWM, OUTPUT);
//     pinMode(MOTOR_ENB, OUTPUT);
//     pinMode(ENCODER_A, INPUT);
//     pinMode(ENCODER_B, INPUT);
//     digitalWrite(MOTOR_ENB, HIGH); // Enable motor driver
    
//     // Configure motor PWM for 18kHz
//     ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, 8); // Channel 2, 18kHz, 8-bit resolution
//     ledcAttachPin(MOTOR_PWM, MOTOR_PWM_CHANNEL); // Attach PWM pin to channel 2
//     attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
// }

// void Motors::setServoAngle(uint8_t angle) {
//     angle = constrain(angle, 0, 100);
//     int mappedAngle = map(angle, 0, 100, 0, 180);
//     servo.write(mappedAngle);
// }

// void Motors::setMotorSpeed(int8_t speed) {
//     speed = constrain(speed, -100, 100);
//     if (speed > 0) {
//         digitalWrite(MOTOR_IN1, HIGH);
//         digitalWrite(MOTOR_IN2, LOW);
//     } else if (speed < 0) {
//         digitalWrite(MOTOR_IN1, LOW);
//         digitalWrite(MOTOR_IN2, HIGH);
//     } else {
//         digitalWrite(MOTOR_IN1, LOW);
//         digitalWrite(MOTOR_IN2, LOW);
//     }
//     ledcWrite(MOTOR_PWM_CHANNEL, abs(speed) * 255 / 100); // Use LEDC channel 1 for PWM
// }

// void Motors::moveDistance(float cm, int8_t speed) {
//     speed = constrain(speed, -100, 100);
//     if (speed == 0) return; // Prevent movement if speed is 0
//     float wheelCircumference = PI * WHEEL_DIAMETER_CM;
//     long targetPulses = (cm / wheelCircumference) * PULSES_PER_REVOLUTION;
//     encoderCount = 0;
//     setMotorSpeed(speed);
//     while (abs(encoderCount) < abs(targetPulses)) {
//         delay(10);
//     }
//     stop();
// }

// void Motors::stop() {
//     digitalWrite(MOTOR_IN1, LOW);
//     digitalWrite(MOTOR_IN2, LOW);
//     ledcWrite(MOTOR_PWM_CHANNEL, 0); // Stop PWM on channel 1
// }

// void Motors::break() {
//     digitalWrite(MOTOR_IN1, LOW);
//     digitalWrite(MOTOR_IN2, LOW);
//     ledcWrite(MOTOR_PWM_CHANNEL, 255); // Hard break PWM on channel 1
// }

// long Motors::getEncoderCount() {
//     return encoderCount;
// }