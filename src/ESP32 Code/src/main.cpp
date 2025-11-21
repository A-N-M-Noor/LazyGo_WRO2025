#include <Arduino.h>

#include <cmath>

#include "OLED.h"
#include "battery.h"
#include "bno.h"
#include "io_pins.h"
#include "motors.h"
#include "parking_operations.h"

#define ENABLE_HW_TEST 0  // Set to 1 to enable hardware test mode
#define COMM_SER Serial1

bool IR_Enabled = false;
float IR_VAL_3 = 4096;
int IR_THRESH_3 = 20;
float IR_VAL_4 = 4096;
int IR_THRESH_4 = 20;

const float TPM = 1873.0;  // Ticks per meter for the wheel encoders, defined here for convenience

Motors motors;
int cam_current = CAM_SERVO_CENTER_US;

float spdMult = 0.75;
int srlToSpd = 1000;
int spd = 0;
int str_angle = SERVO_CENTER_US;  // Servo pulse width in microseconds
long lastEncoderPos = 0;
long currentEncoderPos = 0;
float headingPrev = 0.0;
float Current_angle = 0.0;

float posX = 0.0, posY = 0.0;
float sectionHeading = 0.0;
int dstL = 0, dstFL = 0, dstF = 0, dstFR = 0, dstR = 0;
float moveSideDist = 35.0;
float parkFrontDist[] = {85, 150};
bool park_is_back = false;
bool turning = false;
int turnCount = 0;

long LEDtmr = 0;
long srlTmr = 0;
long stopTmr = 0;
bool stop_bot = 0;
bool running = false;
String command = "none";
// Debug helper to track command transitions
void setCommand(const String &newCmd, const char *origin){
    if(command != newCmd){
        COMM_SER.print(">>CMD_CHANGE: ");
        COMM_SER.print(command);
        COMM_SER.print(" -> ");
        COMM_SER.print(newCmd);
        COMM_SER.print("  @ ");
        COMM_SER.println(origin);
    }
    command = newCmd;
}

float tomove = 0.0;
int key = 0;

long lastOdomSent = 0;
#define odomSendInterval 50

float lerp(float a, float b, float t){
    return a + (b-a)*t;
}

void srl() {
    if (running && millis() - lastOdomSent > odomSendInterval) {
        COMM_SER.printf("[%f,%f,%f]\n", -posY, posX, -heading);
        lastOdomSent = millis();
    }

    while (COMM_SER.available()) {
        int v = COMM_SER.read();
        if(command == "none"){
            LEDtmr = millis();
        }
        if (v == 1) {
            setCommand("right45","srl:1");
        } else if (v == 2) {
            setCommand("left45","srl:2");
        } else if (v == 3) {
            setCommand("straight","srl:3");
        } else if (v == 4) {
            setCommand("passTurn","srl:4");
        } else if (v == 5) {
            setCommand("go","srl:5");
        } 

        else if (v == 6) {
            setCommand("moveRight","srl:6");
            park_is_back = false;
            motors.hardbreak_enabled(false);
        } else if (v == 8) {
            setCommand("moveLeft","srl:8");
            park_is_back = true;
            motors.hardbreak_enabled(false);
        } 
        
        
        else if (v == 30){
            bnoResetNormalize();
        }
        else if (v == 47){
            LEDtmr = millis();
        }
        else if (v == 49) {
            tone(BUZZER_PIN, BUZZ_MID, 250);
        }
        else if (v < 50) {
            key = v;
        }

        if (key != 0 && v >= 50) {
            if (key == 15) {
                spd = map(v, 50, 250, -srlToSpd, srlToSpd);
            }
            if (key == 16) {
                str_angle = map(v, 50, 250, SERVO_MIN_US, SERVO_MAX_US);
            }
            if (key == 17) {
                int ang = constrain(v - 50, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX);

                int t_us = map(ang, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US);
                motors.setCamServoUs(t_us);
            }

            if(key == 31){
                dstL = v - 50;
            }
            if(key == 32){
                dstFL = v - 50;
            }
            if(key == 33){
                dstF = v - 50;
            }
            if(key == 34){
                dstFR = v - 50;
            }
            if(key == 35){
                dstR = v - 50;
            }
            key = 0;
        }
    }
}

void odometry() {
    if(IR_Enabled){
        IR_VAL_3 = lerp(IR_VAL_3, analogRead(IR3_PIN), 0.1);
        IR_VAL_4 = lerp(IR_VAL_4, analogRead(IR4_PIN), 0.1);
    }
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
        odometry();
        srl();

        if (millis() - LEDtmr < 200) {
            digitalWrite(STATUS_LED, HIGH);
        } else {
            digitalWrite(STATUS_LED, LOW);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void enableIR(){
    IR_Enabled = true;
    digitalWrite(IR_EN, HIGH);
}
void disableIR(){
    IR_Enabled = false;
    digitalWrite(IR_EN, LOW);
}

void startSerialReadTask() {
    xTaskCreatePinnedToCore(srlRead, "SerialTask", 4096, NULL, 10, NULL, 1);  // Core 1, high priority
}

void setup() {
    initBNO();
    startBNOCalcTask();
    initADC();

    Serial1.begin(115200, SERIAL_8N1, 19, 23);
    Serial.begin(115200);
    enableIR();
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
    tone(BUZZER_PIN, BUZZ_HIGH, 250);
    tone(BUZZER_PIN, BUZZ_MID, 250);
    noTone(BUZZER_PIN);

    COMM_SER.println(F("CONF_CAM"));

    while (digitalRead(BUTTON_PIN) == HIGH) {
        //bnoCalc();
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks for 100 milliseconds
    }
    disableIR();
    while (digitalRead(BUTTON_PIN) == LOW) {
        //bnoCalc();
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to other tasks for 100 milliseconds
    }
    bnoCalcOffset(200);    // Adjust offset as soon as button is released
    
    tone(BUZZER_PIN, BUZZ_HIGH, 500);

    // move_pos(1.0);
    // COMM_SER.println(F("Boot"));
    
    COMM_SER.println(F("Boot"));
    // command = "park_fin_R";
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

void paark(int dir){
    // enableIR();

    motors.setServoUs(SERVO_CENTER_US);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    move_pos(0.15, 0.0);
    // turn_angle_opp(22.5*dir);
    // turn_angle(45*dir);
    // turn_angle_opp(67.5*dir);
    // turn_angle(75*dir);
    // turn_angle_opp(90*dir);
    // turn_angle(90*dir);

    turn_angle_opp(15*dir);
    turn_angle(30*dir);
    turn_angle_opp(45*dir);
    turn_angle(60*dir);
    turn_angle_opp(75*dir);
    turn_angle(90*dir);
    turn_angle_opp(90*dir);
    motors.setMotorSpeed(0);
    motors.setServoUs(SERVO_CENTER_US);
}

void turn_div(int dir){
    turn_angle_opp(18*dir);
    turn_angle(36*dir);
    turn_angle_opp(54*dir);
    turn_angle(72*dir);
    turn_angle_opp(90*dir);
}


void loop() {
    displayText(command);
    if (command == "right45") {
        turn_angle(45);
        setCommand("none","loop:right45");
        COMM_SER.println("Turned");
    }

    if (command == "left45") {
        turn_angle(-45);
        setCommand("none","loop:left45");
        COMM_SER.println("Turned");
    }

    if (command == "straight") {
        motors.setServoUs(SERVO_CENTER_US);
        move_pos(0.10);
        turn_angle(0);
        setCommand("none","loop:straight");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
        motors.hardbreak_enabled(true);
    }
    if (command == "passTurn") {
        //bnoCalc();
        if (heading > 0) {
            turn_angle(135);
        } else {
            turn_angle(-135);
        }
        turn_angle(0);
        setCommand("none","loop:passTurn");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
        motors.hardbreak_enabled(true);
    }

    if (command == "go") {
        //bnoCalc();
        motors.setMotorSpeed(spd);
        motors.setServoUs(str_angle);
        // odometry();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // IMPORTANT: Yield to other tasks for 100 milliseconds
    }

    if(command == "moveRight"){
        turn_angle(0);
        turn_angle(45);
        motors.setServoUs(SERVO_CENTER_US);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        COMM_SER.println(">>Front-Right Dist: " + String(dstFR));
        float tomove = dstFR - moveSideDist;
        COMM_SER.println(">>To Move: " + String(tomove));
        // if(tomove > 0){
            tomove = tomove * 1.4142;
            COMM_SER.println(">>Adjusted To Move: " + String(tomove));
            tone(BUZZER_PIN, BUZZ_LOW, 200);
            move_pos(tomove/100);
            tone(BUZZER_PIN, BUZZ_HIGH, 200);
        // }
        setCommand("parkRight","loop:moveRight");
    }
    if(command == "moveLeft"){
        turn_angle(0);
        turn_angle(-45);
        motors.setServoUs(SERVO_CENTER_US);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        COMM_SER.println(">>Front-Left Dist: " + String(dstFL));
        float tomove = dstFL - moveSideDist;
        COMM_SER.println(">>To Move: " + String(tomove));
        // if(tomove > 0){
            tomove = tomove * 1.4142;
            COMM_SER.println(">>Adjusted To Move: " + String(tomove));
            tone(BUZZER_PIN, BUZZ_LOW, 200);
            move_pos(tomove/100);
            tone(BUZZER_PIN, BUZZ_HIGH, 200);
        // }
        setCommand("parkLeft","loop:moveLeft");
    }

    if(command == "parkLeft"){
        displayText("Parking Left");
        turn_angle(0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        motors.setServoUs(SERVO_CENTER_US);

        COMM_SER.println(">>Front Dist: " + String(dstF));
        float toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        COMM_SER.println(">>To Move: " + String(toMove));
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);


        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);

        displayText("Initializing Parking");

        turn_angle_opp(90);
        turn_angle_opp(90);
        move_pos(-0.32, 90.0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(dstL > dstR){
            displayText("park_fin_R");
            setCommand("park_fin_R","loop:parkLeft");
        } else {
            displayText("park_fin_L");
            setCommand("park_fin_L","loop:parkLeft");
        }
    }

    if(command == "parkRight"){
        displayText("Parking Right");
        turn_angle(0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        motors.setServoUs(SERVO_CENTER_US);

        COMM_SER.println(">>Front Dist: " + String(dstF));
        float toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        COMM_SER.println(">>To Move: " + String(toMove));
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);


        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);
        displayText("Initializing Parking");

        turn_angle_opp(-90);
        turn_angle_opp(-90);
        move_pos(-0.32, -90.0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(dstL > dstR){
            displayText("park_fin_R");
            setCommand("park_fin_R","loop:parkRight");
        } else {
            displayText("park_fin_L");
            setCommand("park_fin_L","loop:parkRight");
        }
    }
    // if(command == "park_ready_1"){
    //     turn_angle(0);
    //     COMM_SER.println("Oriented");
    //     setCommand("none","loop:park_ready_1");
    // }
    // if (command == "move_pos_dst"){
    //     motors.setServoUs(SERVO_CENTER_US);
    //     move_pos(tomove);
    //     COMM_SER.println("ParkReady");
    //     setCommand("none","loop:move_pos_dst");
    // }
    // if (command == "parkL") {
    //     turn_div(1);
    //     // head_into(90);
    //     move_pos(-1.0);
    //     bnoCalcOffset(500);
    //     setCommand("none","loop:parkL");
    //     COMM_SER.println("Inside");
    // }
    // if (command == "parkR") {
    //     turn_div(-1);
    //     move_pos(-1.0);
    //     bnoCalcOffset(500);
    //     setCommand("none","loop:parkR");
    //     COMM_SER.println("Inside");
    // }

    if(command == "park_fin_R"){
        bnoCalcOffset(500);
        COMM_SER.println(">>Parking Right");
        paark(-1);
        COMM_SER.println("Parked");
        command = "none";
    }
    if(command == "park_fin_L"){
        bnoCalcOffset(500);
        COMM_SER.println(">>Parking Left");
        paark(1);
        COMM_SER.println("Parked");
        command = "none";
    }

    // if (command == "into") {
    //     head_into();
    //     command = "none";
    // }
    if (command == "none") {
        motors.setMotorSpeed(0);
        motors.setServoUs(SERVO_CENTER_US);
    }
}
