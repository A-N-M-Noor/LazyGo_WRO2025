// #include <Arduino.h>

#include <Arduino.h>
#include "IMU.h"
#include "OLED.h"
#include "battery.h"
#include "motors.h"

Motors motors;

int spd = 0;
int str_angle = 0;
long lastEncoderPos = 0;
long currentEncoderPos = 0;
float Previous_angle = 0.0;
float Current_angle = 0.0;

float posX = 0.0, posY = 0.0;

long srlTmr = 0;
long stopTmr = 0;
bool stop_bot = 0;

#define btn 14
#define green 12
void srl() {
    /*if (Serial.available() > 0)
    {
        srlTmr = millis();
        String input = Serial.readStringUntil('\n');
        input.trim();

        if(input == "reset"){
            posX = 0.0;
            posY = 0.0;
        }

        int commaindex = input.indexOf(',');
        if (commaindex <= 0)
        {
            return;
        }
        String xraw = input.substring(0, commaindex);
        String yraw = input.substring(commaindex + 1);

        int x = xraw.toInt();
        int y = yraw.toInt();

        if (x >= -100 && x <= 100)
        {
            if (x == 0)
            {
                motors.hardBreak();
                spd = 0;
            }
            else
            {
                spd = map(x, -100, 100, -255, 255); // Map to servo angle range
                // Serial.println(spd);
            }
        }
        if (y >= -100 && y <= 100)
        {
            if (y == 0)
            {
                str_angle = 1125; // Center position for servo
                motors.setServoUs(str_angle);
            }
            else
            {
                str_angle = map(y, -100, 100, 550, 1700); // Map to motor speed range
                motors.setServoUs(str_angle);
            }
        }
    }*/

    while (Serial.available()) {
        int v = Serial.read();
        srlTmr = millis();
        if (v >= 50 && v <= 150) {
            if (v == 100) {
                // motors.hardBreak();
                spd = 0;
            } else {
                spd = map(v, 50, 150, -255, 255);
            }
        } else if (v > 150 && v <= 250) {
            if (v == 200) {
                str_angle = 1125;  // Center position for servo
                motors.setServoUs(str_angle);
            } else {
                str_angle = map(v, 151, 250, 550, 1700);  // Map to motor speed range
            }
        }
    }
}

void odometry() {
    currentEncoderPos = motors.getCurrentPosition();
    float deltaEncoderPos = currentEncoderPos - lastEncoderPos;
    float ang = (continuousYaw - Previous_angle) / 2;
    Previous_angle = continuousYaw;
    lastEncoderPos = currentEncoderPos;

    if (abs(deltaEncoderPos) > 0) {
        float deltaX = deltaEncoderPos * cos(ang * DEG_TO_RAD);
        float deltaY = deltaEncoderPos * sin(ang * DEG_TO_RAD);

        posX += deltaX;
        posY += deltaY;

        Serial.print(currentEncoderPos);
        Serial.print("[");
        Serial.print(posX);
        Serial.print(",");
        Serial.print(posY);
        Serial.println("]");
    }
}

void keyboard_Works() {
    if (Serial.available()) {
        srlTmr = millis();
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0)
            return;

        char command = input.charAt(0);
        String value = input.substring(1);

        switch (command) {
            case 's': {  // Servo angle
                int angle = value.toInt();
                if (angle >= 0 && angle <= 100) {
                    motors.setServoAngle(angle);
                    Serial.print("Servo set to ");
                    Serial.print(angle);
                } else {
                    Serial.println("Invalid servo angle (0-100)");
                }
                break;
            }
            case 'm': {  // Motor speed
                int speed = value.toInt();
                if (speed >= -1000 && speed <= 1000) {
                    motors.setMotorSpeed(speed);
                    Serial.print("Motor speed set to ");
                    Serial.println(speed);
                } else {
                    Serial.println("Invalid speed (-1000 to 1000)");
                }
                break;
            }
            case 'd': {  // Move distance with speed
                int commaIndex = value.indexOf(',');
                if (commaIndex != -1) {
                    float distance = value.substring(0, commaIndex).toFloat();
                    int speed = value.substring(commaIndex + 1).toInt();
                    if (speed >= -1000 && speed <= 1000 && speed != 0) {
                        motors.moveDistance(distance, speed);
                        Serial.print("Moved ");
                        Serial.print(distance);
                        Serial.print(" cm at speed ");
                        Serial.println(speed);
                    } else {
                        Serial.println("Invalid distance (>0) or speed (-1000 to 1000, non-zero)");
                    }
                } else {
                    Serial.println("Invalid format. Use d<distance>,<speed>");
                }
                break;
            }
            case 'x': {  // Stop
                motors.stop();
                Serial.println("Motor stopped");
                break;
            }
            default:
                Serial.println("Unknown command. Use s, m, d, or x");
        }
        Serial.print("Encoder count: ");
        Serial.println(motors.getEncoderCount());
    }
}

void srlRead(void *pvParameters) {
    while (true) {
        srl();
        if (millis() - srlTmr < 100) {
            digitalWrite(green, HIGH);
        } else {
            digitalWrite(green, LOW);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks for 50 milliseconds
    }
}

void startSerialReadTask() {
    xTaskCreatePinnedToCore(srlRead, "SerialTask", 4096, NULL, 10, NULL, 1);  // Core 1, high priority
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    initADC();  // Initialize battery ADC
    initIMU();  // Initialize MPU6050
    // initOLED(); // Initialize OLED
    motors.begin();
    startTime = millis();  // Set start time
    pinMode(green, OUTPUT);
    // Start FreeRTOS tasks
    startIMUTask();  // MPU task on Core 0
    // startOLEDDisplayTask(); // OLED task on Core 1
    startSerialReadTask();  // Serial read task on Core 1
    motors.setServoUs(1125);

    pinMode(btn, INPUT_PULLUP);  // Button pin for manual control

    while (digitalRead(btn) == HIGH) {
        // Wait for button press to start
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks
    }
    while (digitalRead(btn) == LOW) {
        // Wait for button release to start
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks
    }
    motors.setServoUs(1125);                       // Center servo position
    motors.run(0);                                 // Stop motors initially
    lastEncoderPos = motors.getCurrentPosition();  // Initialize last encoder position
    Previous_angle = continuousYaw;   // Initialize previous angle
    posX = 0.0;                                    // Initialize position X
    posY = 0.0;                                    // Initialize position Y
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void loop() {
    // srl();

    motors.run(spd);
    motors.setServoUs(str_angle);
    if (millis() - srlTmr > 1000) {
        spd = 0;
        str_angle = 1125;
        motors.run(0);
        motors.setServoUs(1125);
    }
    // Serial.print(spd);
    // Serial.print(",");
    // Serial.println(str_angle);
    odometry();
    if (stop_bot) {
        if (millis() - stopTmr > 2000) {
            motors.run(0);
            motors.setServoUs(1125);  // Center servo position
            delay(100);
            ESP.restart();  // Restart if MPU data is invalid
        }
    }

    if (abs(continuousYaw) > 1000 && stop_bot == 0) {
        stopTmr = millis();
        stop_bot = 1;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // IMPORTANT: Yield to other tasks for 100 milliseconds
}
