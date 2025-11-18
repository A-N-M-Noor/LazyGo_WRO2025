#include <Arduino.h>

#include <cmath>

#include "OLED.h"
#include "battery.h"
#include "bno.h"
#include "io_pins.h"
#include "motors.h"
#include "parking_operations.h"

#define ENABLE_HW_TEST 0  // Set to 1 to enable hardware test mode
#define COMM_SER Serial

const float TPM = 1775.0;  // Ticks per meter for the wheel encoders, defined here for convenience

Motors motors;
int cam_current = CAM_SERVO_CENTER_US;

float spdMult = 0.75;
int spd = 0;
int str_angle = SERVO_CENTER_US;  // Servo pulse width in microseconds
long lastEncoderPos = 0;
long currentEncoderPos = 0;
float headingPrev = 0.0;
float Current_angle = 0.0;

float posX = 0.0, posY = 0.0;
float sectionHeading = 0.0;
bool turning = false;
int turnCount = 0;
int laps = 3;

long srlTmr = 0;
long stopTmr = 0;
bool stop_bot = 0;
bool running = false;
String command = "none";

int key = 0;

void srl() {
    if (running) {
        Serial.printf("[%f,%f,%f]\n", -posY, posX, -heading);
        COMM_SER.printf("[%f,%f,%f]\n", -posY, posX, -heading);
    }

    while (COMM_SER.available()) {
        int v = COMM_SER.read();
        srlTmr = millis();
        if (v == 1) {
            command = "right45";
        } else if (v == 2) {
            command = "left45";
        } else if (v == 3) {
            command = "straight";
        } else if (v == 4) {
            command = "passTurn";
        } else if (v == 5) {
            command = "go";
        } else if (v == 6) {
            command = "parkR";
        } else if (v == 7) {
            command = "parkL";
        }
        else if (v < 50) {
            key = v;
        }

        if (key != 0 && v >= 50) {
            if (key == 15) {
                spd = map(v, 50, 250, -600, 600);
            }
            if (key == 16) {
                str_angle = map(v, 50, 250, SERVO_MIN_US, SERVO_MAX_US);
            }
            if (key == 17) {
                int ang = constrain(v - 50, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX);

                int t_us = map(ang, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US);
                motors.setCamServoUs(t_us);
            }
            key = 0;
        }
    }
}

void odometry() {
    currentEncoderPos = motors.getEncoderCount();
    float deltaEncoderPos = (currentEncoderPos - lastEncoderPos) / TPM;
    float ang = (heading + headingPrev) / 2;
    headingPrev = heading;
    lastEncoderPos = currentEncoderPos;

    if (abs(deltaEncoderPos) > 0) {
        float deltaX = deltaEncoderPos * cos(ang * DEG_TO_RAD);
        float deltaY = deltaEncoderPos * sin(ang * DEG_TO_RAD);

        posX += deltaX;
        posY += deltaY;
    }
}

void srlRead(void* pvParameters) {
    while (true) {
        srl();
        if (millis() - srlTmr < 100) {
            digitalWrite(STATUS_LED, HIGH);
        } else {
            digitalWrite(STATUS_LED, LOW);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void startSerialReadTask() {
    xTaskCreatePinnedToCore(srlRead, "SerialTask", 4096, NULL, 10, NULL, 1);  // Core 1, high priority
}

void setup() {
    initBNO();
    startBNOCalcTask();
    initADC();

    Serial1.begin(115200, SERIAL_8N1, 23, 19);
    Serial.begin(115200);

    initOLED();
    startOLEDDisplayTask();
    displayText("Wait...");

    motors.begin();
    initIO();

    while (COMM_SER.available() > 0)
    {
        COMM_SER.read();
    }

    startSerialReadTask();  // Serial read task on Core 1
    motors.setServoUs(SERVO_CENTER_US);
    motors.setCamServoUs(CAM_SERVO_CENTER_US);
    bnoCalcOffset(1500);
    // IO initialization (pin modes + startup tones) handled in initIO()
    displayText("All okay!");
    // Startup buzzer pattern
    tone(BUZZER_PIN, BUZZ_LOW, 250);
    tone(BUZZER_PIN, BUZZ_HIGH, 250);
    tone(BUZZER_PIN, BUZZ_LOW, 500);
    tone(BUZZER_PIN, BUZZ_HIGH, 500);
    noTone(BUZZER_PIN);
    while (digitalRead(BUTTON_PIN) == HIGH) {
        //bnoCalc();
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks for 100 milliseconds
    }
    while (digitalRead(BUTTON_PIN) == LOW) {
        //bnoCalc();
        tone(BUZZER_PIN, BUZZ_HIGH, 200);
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks for 100 milliseconds
    }
    bnoCalcOffset(200);    // Adjust offset as soon as button is released

    Serial.println(F("Boot"));
    // command = "parkL";
    displayText("");
    motors.setServoUs(SERVO_CENTER_US);         // Center servo position
    motors.setMotorSpeed(0);                              // Stop motors initially
    lastEncoderPos = motors.getEncoderCount();  // Initialize last encoder position
    headingPrev = heading;                      // Initialize previous angle
    posX = 0.0;                                 // Initialize position X
    posY = 0.0;                                 // Initialize position Y

    sectionHeading = heading;
    startTime = millis();
    running = true;

    #if ENABLE_HW_TEST
        randomSeed(esp_random());
        while (true) {
            motors.run(random(-255, 256));
            motors.setServoUs(random(SERVO_MIN_US, SERVO_MAX_US + 1));
            motors.setCamServoUs(random(CAM_SERVO_MIN_US, CAM_SERVO_MAX_US + 1));
            if (random(0, 1000) < 50) {
                tone(BUZZER_PIN, BUZZ_LOW, 500);
                digitalWrite(STATUS_LED, HIGH);
                delay(random(300, 800));
                digitalWrite(STATUS_LED, LOW);
            }
            vTaskDelay(random(300, 1500) / portTICK_PERIOD_MS);
        }
    #endif
}

void paaark(int dir) {
    //bnoCalc();
    float dr = fmod(heading, 360.0);

    float err = -dir * 90 + dr;
    float target = dr + dir * 10;
    bool frd = false;
    motors.setServoUs(SERVO_CENTER_US);
    move_pos(0.2);
    while (dr > -85 && dr < 85) {
        //bnoCalc();
        odometry();
        err = -dir * 90 + dr;
        COMM_SER.println(err);
        if (frd) {
            turn_angle(target);
        } else {
            turn_angle_opp(target);
        }
        frd = !frd;
        target = dr + dir * 10;
        delay(10);
    }
    command = "into";
    motors.setMotorSpeed(0);
    motors.setServoUs(SERVO_CENTER_US);
}

void loop() {
    displayText(command);
    if (command == "right45") {
        turn_angle(45);
        command = "none";
        Serial.println("Turned");
        COMM_SER.println("Turned");
    }

    if (command == "left45") {
        turn_angle(-45);
        command = "none";
        Serial.println("Turned");
        COMM_SER.println("Turned");
    }

    if (command == "straight") {
        motors.setServoUs(SERVO_CENTER_US);
        move_pos(0.10);
        turn_angle(0);
        command = "none";
        Serial.println("Done");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
    }
    if (command == "passTurn") {
        //bnoCalc();
        if (heading > 0) {
            turn_angle(135);
        } else {
            turn_angle(-135);
        }
        turn_angle(0);
        command = "none";
        Serial.println("Done");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
    }

    if (command == "go") {
        //bnoCalc();
        motors.setMotorSpeed(spd);
        motors.setServoUs(str_angle);
        odometry();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // IMPORTANT: Yield to other tasks for 100 milliseconds
    }

    if (command == "parkR") {
        paaark(1);
    }
    if (command == "parkL") {
        paaark(-1);
    }

    if (command == "into") {
        head_into();
        command = "none";
    }
    if (command == "none") {
        motors.setMotorSpeed(0);
        motors.setServoUs(SERVO_CENTER_US);
    }
}
