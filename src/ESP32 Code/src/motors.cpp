#include "motors.h"

#include <driver/mcpwm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

extern float spdMult;
// Initialize static member
Motors* Motors::instance = nullptr;

void IRAM_ATTR Motors::encoderISR() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = micros();
    if (interruptTime - lastInterruptTime > 10)  // debounce 10us
    {
        if (instance) {
            if (digitalRead(ENCODER_B) == HIGH) {
                instance->encoderCount++;
            } else {
                instance->encoderCount--;
            }
            // Position control removed; keep only encoder count
        }
    }
    lastInterruptTime = interruptTime;
}

void Motors::control_enabled(bool en){
    do_control = en;
}

// FreeRTOS task for PID speed and position control
void Motors::speedControlTask(void* pvParameters) {
    // Simple PI speed controller using encoder feedback (mm/s)
    // Runs periodically with minimal blocking via vTaskDelayUntil.

    // Access global ticks-per-meter defined in main.cpp
    extern const float TPM;  // ticks per meter

    // Controller parameters (tuned conservatively)
    const TickType_t periodTicks = pdMS_TO_TICKS(80);  // 12.5 Hz control
    const float dt = 0.080f;                           // 80 ms
    const float kp = 0.04f;                           // proportional gain
    const float ki = 0.35f;                            // integral gain (per second)
    const int maxPWM = 255;
    const int minPWM = 18;        // overcome static friction
    const int slewPerCycle = 200;  // limit change per cycle 
    const float FF_GAIN = 0.0f;   // feedforward: gives a beginning push, set to 0.0 to disable, (0.10-0.40) might help with low speeds  but too much can make car jump

    TickType_t lastWake = xTaskGetTickCount();
    Motors* self = static_cast<Motors*>(pvParameters);
    long lastTicks = self->encoderCount;
    float integrator = 0.0f;
    int cmdPWM = 0;  // command in -255..255

    while (true) {
        if(!self->do_control){
            vTaskDelay(periodTicks);
            continue;
        }
        // Measure actual speed (mm/s)
        long ticksNow = self->encoderCount;
        long dTicks = ticksNow - lastTicks;
        lastTicks = ticksNow;

        float delta_m = (float)dTicks / TPM;        // meters in dt
        float meas_mms = (delta_m * 1000.0f) / dt;  // mm/s

        // Target speed in mm/s (no position control)
        float target_mms = self->target_speed_mms;

        // Basic PI control
        float err = target_mms - meas_mms;
        integrator += err * dt;  // integrate error (mm)
        // Anti-windup clamp
        if (integrator > 5000.0f) integrator = 5000.0f;
        if (integrator < -5000.0f) integrator = -5000.0f;

        // PI correction (integrator in mm*s)
        float piTerm = kp * err + ki * integrator;
        // Feedforward term based on desired speed
        int ff = (int)lroundf(FF_GAIN * target_mms);  // signed
        // Desired PWM = feedforward + PI correction
        int desiredPWM = ff + (int)lroundf(piTerm);

        // Near-zero target -> stop cleanly
        if (fabsf(target_mms) < 3.0f) {
            desiredPWM = 0;
            integrator = 0;
        }

        // Apply deadband when commanding motion
        if (desiredPWM != 0) {
            if (desiredPWM > 0 && desiredPWM < minPWM) desiredPWM = minPWM;
            if (desiredPWM < 0 && desiredPWM > -minPWM) desiredPWM = -minPWM;
        }

        // Slew-rate limit
        int deltaCmd = desiredPWM - cmdPWM;
        if (deltaCmd > slewPerCycle) deltaCmd = slewPerCycle;
        if (deltaCmd < -slewPerCycle) deltaCmd = -slewPerCycle;
        cmdPWM += deltaCmd;

        // Clamp to valid range
        if (cmdPWM > maxPWM) cmdPWM = maxPWM;
        if (cmdPWM < -maxPWM) cmdPWM = -maxPWM;

        // Drive motor
        self->run(cmdPWM);

        // Keep timing stable without blocking other tasks
        vTaskDelayUntil(&lastWake, periodTicks);
    }
}

void Motors::begin() {
    instance = this;       // Set the static instance pointer
    encoderCount = 0;      // Initialize encoder count
    target_speed_mms = 0;  // Initialize target speed
    do_control = true;
    // Position control removed

    // Configure MCPWM for steering servo (Timer 0, Operator A)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);
    mcpwm_config_t pwm_config = {
        .frequency = 50,                   // 50 Hz for servo
        .cmpr_a = 0,                       // Initial duty cycle for operator A
        .cmpr_b = 0,                       // Initial duty cycle for operator B (not used)
        .duty_mode = MCPWM_DUTY_MODE_0,    // Duty cycle mode
        .counter_mode = MCPWM_UP_COUNTER,  // Counter mode
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Configure MCPWM for camera servo (Timer 1, Operator A)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, CAM_SERVO_PIN);
    mcpwm_config_t cam_pwm_config = {
        .frequency = 50,                   // 50 Hz for servo
        .cmpr_a = 0,                       // Initial duty cycle for operator A
        .cmpr_b = 0,                       // Initial duty cycle for operator B (not used)
        .duty_mode = MCPWM_DUTY_MODE_0,    // Duty cycle mode
        .counter_mode = MCPWM_UP_COUNTER,  // Counter mode
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cam_pwm_config);

    // Configure motor pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    digitalWrite(MOTOR_ENB, HIGH);  // Enable motor driver

    // Disaled ledcSetup/ledcAttachPin/ledcWrite
    // Configure motor PWM for 18kHz
    // ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, 8);  // Channel 0, 18kHz, 8-bit resolution
    // ledcAttachPin(MOTOR_PWM, MOTOR_PWM_CHANNEL);      // Attach PWM pin to channel 0
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    // Add MCPWM for motor
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, MOTOR_PWM);  // Use Timer 2
    mcpwm_config_t motor_pwm = {
        .frequency = 18000,  // 18kHz
        .cmpr_a = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &motor_pwm);

    // Create speed control task pinned to core 0
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<Motors*>(arg)->speedControlTask(arg); },
        "SpeedControlTask",
        4096,
        this,
        1,
        nullptr,
        0);
}

void Motors::setServoUs(uint32_t pulse_width_us) {
    pulse_width_us = constrain(pulse_width_us, SERVO_MIN_US, SERVO_MAX_US);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width_us);
}

void Motors::setServoAngle(uint8_t angle0to100) {
    angle0to100 = constrain(angle0to100, 0, 100);
    uint32_t pulse_width_us;
    if (angle0to100 <= 50) {
        // Map 0–50 to SERVO_MIN_US–SERVO_CENTER_US
        pulse_width_us = map(angle0to100, 0, 50, SERVO_MIN_US, SERVO_CENTER_US);
    } else {
        // Map 50–100 to SERVO_CENTER_US–SERVO_MAX_US
        pulse_width_us = map(angle0to100, 50, 100, SERVO_CENTER_US, SERVO_MAX_US);
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width_us);
}

void Motors::setCamServoUs(uint32_t pulse_width_us) {
    pulse_width_us = constrain(pulse_width_us, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, pulse_width_us);
}

void Motors::setMotorSpeed(float speed_mms) {
    target_speed_mms = speed_mms;  // Update target speed for the task
    if(speed_mms == 0){
        hardBreak();
    }
}

// Position control removed

void Motors::run(int spd_vlu) {
    // Direct PWM command; do not scale or modify speed target here
    if (spd_vlu == 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    } else {
        if (spd_vlu < 0) {
            digitalWrite(MOTOR_IN1, HIGH);
            digitalWrite(MOTOR_IN2, LOW);
        } else {
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, HIGH);
        }
        float duty = abs(spd_vlu) / 255.0 * 100.0;  // 0–100%
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty);
    }
}

void Motors::stop() {
    target_speed_mms = 0;  // Signal task to stop
    run(0);
}

// void Motors::hardBreak() {
//     target_speed_mms = 0;  // Signal task to stop
//     digitalWrite(MOTOR_IN1, LOW);
//     digitalWrite(MOTOR_IN2, LOW);
//     mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 100);  // Hard brake
// }

void Motors::hardBreak() {
    target_speed_mms = 0;  // Signal task to stop
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 100);  // Hard brake
}

long Motors::getEncoderCount() {
    return encoderCount;
}

// getCurrentPosition removed with position control