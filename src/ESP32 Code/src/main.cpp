#include <Arduino.h>
#include <NewPing.h>
#include <cmath>
#include "MPU.h"
#include "OLED.h"
#include "battery.h"
#include "motors.h"
#include "bno.h"
#include "parking_operations.h"

float TPM = 1955.0;

Motors motors;
int cam_current = CAM_SERVO_CENTER_US;

float spdMult = 0.75;
int spd = 0;
int str_angle = SERVO_CENTER_US; // Servo pulse width in microseconds
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


#define btn 0
#define green 2
#define buzzer 4


void srl()
{   
    if(running){
        Serial.printf("[%f,%f,%f]\n", -posY, posX, -heading);
    }
    
    while (Serial.available())
    {
        int v = Serial.read();
        srlTmr = millis();
        if(v >= 15 && v < 50){
            key = v;
        }

        else if(v == 1){
            command = "right45";
        }
        else if(v == 2){
            command = "left45";
        }
        else if(v == 3){
            command = "straight";
        }
        else if(v == 4){
            command = "passTurn";
        }
        else if(v == 5){
            command = "go";
        }
        else if(v == 6){
            command = "parkR";
        }
        else if(v == 7){
            command = "parkL";
        }

        if(key != 0 && v >= 50){
            if(key == 15){
                spd = map(v, 50, 250, -255, 255);
            }
            if(key == 16){
                str_angle = map(v, 50, 250, SERVO_MIN_US, SERVO_MAX_US);
            }
            if(key == 17){
                int ang = v - 50;

                int t_us = map(ang, 0, 180, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US);
                motors.setCamServoUs(t_us);

            }
            key = 0;
        }
        
    }
}

void odometry()
{
    currentEncoderPos = motors.getEncoderCount();
    float deltaEncoderPos = (currentEncoderPos - lastEncoderPos)/TPM;
    float ang = (heading + headingPrev) / 2;
    headingPrev = heading;
    lastEncoderPos = currentEncoderPos;

    if (abs(deltaEncoderPos) > 0)
    {
        float deltaX = deltaEncoderPos * cos(ang * DEG_TO_RAD);
        float deltaY = deltaEncoderPos * sin(ang * DEG_TO_RAD);

        posX += deltaX;
        posY += deltaY;
    }
}

void srlRead(void *pvParameters)
{
    while (true)
    {
        srl();
        if (millis() - srlTmr < 100)
        {
            digitalWrite(green, HIGH);
        }
        else
        {
            digitalWrite(green, LOW);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void startSerialReadTask()
{
    xTaskCreatePinnedToCore(srlRead, "SerialTask", 4096, NULL, 10, NULL, 1); // Core 1, high priority
}

void setup()
{   

    initBNO();

    
    Serial.begin(115200);
    while (!Serial)
        ;

    initOLED();
    startOLEDDisplayTask();
    displayText("Wait...");

    bnoCalc();
    bnoCalcOffset(1500);

    motors.begin();
    pinMode(green, OUTPUT);
    pinMode(buzzer, OUTPUT);

    while (Serial.available() > 0)
    {
        Serial.read();
    }

    startSerialReadTask(); // Serial read task on Core 1
    motors.setServoUs(SERVO_CENTER_US);
    motors.setCamServoUs(CAM_SERVO_CENTER_US);

    pinMode(btn, INPUT_PULLUP); // Button pin for manual control

    digitalWrite(green, HIGH);
    displayText("All okay!");
    // motors.run(255);
    while (digitalRead(btn) == HIGH)
    {
        bnoCalc();
        vTaskDelay(50 / portTICK_PERIOD_MS); // Yield to other tasks for 100 milliseconds
    }
    while (digitalRead(btn) == LOW)
    {
        bnoCalc();
        vTaskDelay(50 / portTICK_PERIOD_MS); // Yield to other tasks for 100 milliseconds
    }
    Serial.println(F("Boot"));
    // command = "parkL";
    displayText("");
    motors.setServoUs(SERVO_CENTER_US);                      // Center servo position
    motors.run(0);                                // Stop motors initially
    lastEncoderPos = motors.getEncoderCount();    // Initialize last encoder position
    headingPrev = heading;                        // Initialize previous angle
    posX = 0.0;                                   // Initialize position X
    posY = 0.0;                                   // Initialize position Y
    
    sectionHeading = heading;
    startTime = millis();
    running = true;
}

void paaark(int dir){
    bnoCalc();
    float dr = fmod(heading, 360.0);

    float err = -dir*90 + dr;
    float target = dr + dir*10;
    bool frd = false;
    motors.setServoUs(SERVO_CENTER_US);
    move_pos(0.2);
    while(dr > -85 && dr < 85){
        bnoCalc();
        odometry();
        err = -dir*90 + dr;
        Serial.println(err);
        if(frd){
            turn_angle(target);
        }
        else{
            turn_angle_opp(target);
        }
        frd = !frd;
        target = dr + dir*10;
        delay(10);
    }
    command = "into";
    motors.run(0);
    motors.setServoUs(SERVO_CENTER_US);
}

void loop()
{   

    displayText(command);
    if(command == "right45"){
        turn_angle(45);
        command = "none";
        Serial.println("Turned");
    }

    if(command == "left45"){
        turn_angle(-45);
        command = "none";
        Serial.println("Turned");
    }

    if(command == "straight"){
        motors.setServoUs(SERVO_CENTER_US);
        move_pos(0.10);
        turn_angle(0);
        command = "none";
        Serial.println("Done");
        posX = 0.0;
        posY = 0.0;
    }
    if(command == "passTurn"){
        bnoCalc();
        if(heading > 0){
            turn_angle(135);
        }
        else{
            turn_angle(-135);
        }
        turn_angle(0);
        command = "none";
        Serial.println("Done");
        posX = 0.0;
        posY = 0.0;
    }

    if(command == "go"){
        bnoCalc();
        motors.run(spd);
        motors.setServoUs(str_angle);
        odometry();
        vTaskDelay(10 / portTICK_PERIOD_MS); // IMPORTANT: Yield to other tasks for 100 milliseconds
    }

    if(command == "parkR"){
        paaark(1);
    }
    if(command == "parkL"){
        paaark(-1);
    }

    if(command == "into"){
        head_into();
        command = "none";
    }
    if(command == "none"){
        motors.run(0);
        motors.setServoUs(SERVO_CENTER_US);
    }
}
