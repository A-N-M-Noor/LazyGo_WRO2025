#include "motors.h"
#include <driver/mcpwm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Initialize static member
Motors *Motors::instance = nullptr;

void IRAM_ATTR Motors::encoderISR()
{
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = micros();
    if (interruptTime - lastInterruptTime > 10) // debounce 10us
    {
        if (instance)
        {
            if (digitalRead(ENCODER_B) == HIGH)
            {
                instance->encoderCount++;
            }
            else
            {
                instance->encoderCount--;
            }
            // Update current position in millimeters
            instance->current_position_mm = (instance->encoderCount * PI * WHEEL_DIAMETER_CM * 10.0) / PULSES_PER_REVOLUTION;
        }
    }
    lastInterruptTime = interruptTime;
}

// FreeRTOS task for PID speed and position control
void Motors::speedControlTask(void *pvParameters)
{
    Motors *motors = (Motors *)pvParameters;
    const uint32_t sample_interval_ms = 100; // Sample every 100 ms
    const float kp_speed = 0.3;              // Proportional gain for speed control
    const float kp_position = 0.5;           // Proportional gain for position control (tuned)
    const float position_tolerance_mm = 1.0; // Stop within 1 mm of target
    float wheel_circumference_mm = PI * WHEEL_DIAMETER_CM * 10.0;
    float pulses_per_mm = PULSES_PER_REVOLUTION / wheel_circumference_mm;
    uint8_t pwm_duty = 128; // Start with 50% duty cycle
    long prev_encoder_count = 0;
    unsigned long last_sample_time = millis();

    while (true)
    {
        unsigned long current_time = millis();
        unsigned long delta_time_ms = current_time - last_sample_time;

        // Calculate actual speed
        long delta_pulses = motors->encoderCount - prev_encoder_count;
        float actual_pulses_per_sec = (delta_pulses * 1000.0) / sample_interval_ms;
        float actual_speed_mms = abs(actual_pulses_per_sec / pulses_per_mm);

        if (motors->use_position_control && abs(motors->target_position_mm - motors->current_position_mm) > position_tolerance_mm)
        {
            // Position control mode
            float position_error = motors->target_position_mm - motors->current_position_mm;
            float commanded_speed_mms = kp_position * position_error;
            // Limit commanded speed to avoid overshooting
            commanded_speed_mms = constrain(commanded_speed_mms, -motors->target_speed_mms, motors->target_speed_mms);

            // Calculate speed error
            float speed_error = commanded_speed_mms - actual_speed_mms;
            int pwm_adjust = (int)(kp_speed * speed_error);
            pwm_duty = constrain(pwm_duty + pwm_adjust, 0, 255);

            // Apply direction and PWM
            if (commanded_speed_mms == 0)
            {
                digitalWrite(MOTOR_IN1, LOW);
                digitalWrite(MOTOR_IN2, LOW);
                ledcWrite(MOTOR_PWM_CHANNEL, 0);
            }
            else
            {
                if (commanded_speed_mms < 0)
                {
                    digitalWrite(MOTOR_IN1, HIGH);
                    digitalWrite(MOTOR_IN2, LOW);
                }
                else
                {
                    digitalWrite(MOTOR_IN1, LOW);
                    digitalWrite(MOTOR_IN2, HIGH);
                }
                ledcWrite(MOTOR_PWM_CHANNEL, pwm_duty);
            }
        }
        else
        {
            // Speed control mode (or stopped)
            if (motors->use_position_control)
            {
                // Reached target position, apply hard brake
                motors->use_position_control = false;
                motors->target_speed_mms = 0;
                motors->target_position_mm = motors->current_position_mm;
                digitalWrite(MOTOR_IN1, LOW);
                digitalWrite(MOTOR_IN2, LOW);
                ledcWrite(MOTOR_PWM_CHANNEL, 255); // Hard brake
            }
            else if (motors->target_speed_mms == 0)
            {
                digitalWrite(MOTOR_IN1, LOW);
                digitalWrite(MOTOR_IN2, LOW);
                ledcWrite(MOTOR_PWM_CHANNEL, 0);
            }
            else
            {
                // Speed control
                float target_speed = motors->target_speed_mms;
                float speed_error = (target_speed > 0 ? target_speed : -target_speed) - actual_speed_mms;
                int pwm_adjust = (int)(kp_speed * speed_error);
                pwm_duty = constrain(pwm_duty + pwm_adjust, 0, 255);

                if (motors->target_speed_mms < 0)
                {
                    digitalWrite(MOTOR_IN1, HIGH);
                    digitalWrite(MOTOR_IN2, LOW);
                }
                else
                {
                    digitalWrite(MOTOR_IN1, LOW);
                    digitalWrite(MOTOR_IN2, HIGH);
                }
                ledcWrite(MOTOR_PWM_CHANNEL, pwm_duty);
            }
        }

        // Update state
        prev_encoder_count = motors->encoderCount;
        last_sample_time = current_time;

        // Delay until next sample
        vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
    }
}

void Motors::begin()
{
    instance = this;              // Set the static instance pointer
    encoderCount = 0;             // Initialize encoder count
    target_speed_mms = 0;         // Initialize target speed
    target_position_mm = 0;       // Initialize target position
    current_position_mm = 0;      // Initialize current position
    use_position_control = false; // Initialize to speed control

    // Configure MCPWM for servo
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);
    mcpwm_config_t pwm_config = {
        .frequency = 50,                  // 50 Hz for servo
        .cmpr_a = 0,                      // Initial duty cycle for operator A
        .cmpr_b = 0,                      // Initial duty cycle for operator B (not used)
        .duty_mode = MCPWM_DUTY_MODE_0,   // Duty cycle mode
        .counter_mode = MCPWM_UP_COUNTER, // Counter mode
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Configure motor pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_ENB, OUTPUT);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    digitalWrite(MOTOR_ENB, HIGH); // Enable motor driver

    // Configure motor PWM for 18kHz
    ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, 8); // Channel 0, 18kHz, 8-bit resolution
    ledcAttachPin(MOTOR_PWM, MOTOR_PWM_CHANNEL);     // Attach PWM pin to channel 0
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    // Create speed control task pinned to core 0
    // xTaskCreatePinnedToCore(
    //     speedControlTask,   // Task function
    //     "SpeedControlTask", // Task name
    //     4096,               // Stack size
    //     this,               // Task parameter (pointer to Motors instance)
    //     1,                  // Priority
    //     NULL,               // Task handle
    //     0                   // Core 0
    // );
}

void Motors::setServoUs(uint32_t pulse_width_us)
{
    pulse_width_us = constrain(pulse_width_us, SERVO_MIN_US, SERVO_MAX_US);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width_us);
}

void Motors::setServoAngle(uint8_t angle0to100)
{
    angle0to100 = constrain(angle0to100, 0, 100);
    uint32_t pulse_width_us;
    if (angle0to100 <= 50)
    {
        // Map 0–50 to SERVO_MIN_US–SERVO_CENTER_US
        pulse_width_us = map(angle0to100, 0, 50, SERVO_MIN_US, SERVO_CENTER_US);
    }
    else
    {
        // Map 50–100 to SERVO_CENTER_US–SERVO_MAX_US
        pulse_width_us = map(angle0to100, 50, 100, SERVO_CENTER_US, SERVO_MAX_US);
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width_us);
}

void Motors::setMotorSpeed(float speed_mms)
{
    target_speed_mms = speed_mms; // Update target speed for the task
    use_position_control = false; // Switch to speed control
}

void Motors::moveDistance(float cm, float speed_mms)
{
    if (speed_mms == 0)
        return;                                             // Prevent movement if speed is 0
    target_speed_mms = abs(speed_mms);                      // Set max speed for position control
    target_position_mm = current_position_mm + (cm * 10.0); // Convert cm to mm and add to current position
    use_position_control = true;                            // Switch to position control
}
void Motors::run(int spd_vlu)
{   
    if (spd_vlu == 0)
    {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL, 0);
    }
    else
    {
        if (spd_vlu < 0)
        {
            digitalWrite(MOTOR_IN1, HIGH);
            digitalWrite(MOTOR_IN2, LOW);
        }
        else
        {
            digitalWrite(MOTOR_IN1, LOW);
            digitalWrite(MOTOR_IN2, HIGH);
        }
        ledcWrite(MOTOR_PWM_CHANNEL, abs(spd_vlu));
    }
}

void Motors::stop()
{
    target_speed_mms = 0;                     // Signal task to stop
    target_position_mm = current_position_mm; // Disable position control
    use_position_control = false;             // Switch to speed control
}

void Motors::hardBreak()
{
    target_speed_mms = 0;                     // Signal task to stop
    target_position_mm = current_position_mm; // Disable position control
    use_position_control = false;             // Switch to speed control
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(MOTOR_PWM_CHANNEL, 255); // Hard brake
}

long Motors::getEncoderCount()
{
    return encoderCount;
}

float Motors::getCurrentPosition()
{
    return current_position_mm;
}