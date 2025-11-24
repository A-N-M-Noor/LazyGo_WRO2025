#include <Arduino.h>
#include <cmath>

#include "OLED.h"
#include "battery.h"
#include "bno.h"
#include "io_pins.h"
#include "motors.h"
#include "parking_operations.h"

// --- Configuration ---
#define ENABLE_HW_TEST 0  // Set to 1 to enable hardware test mode (random movements)
#define COMM_SER Serial1  // Serial port for communication with Raspberry Pi

// --- IR Sensor Globals ---
bool IR_Enabled = false;
float IR_VAL_3 = 4096;
int IR_THRESH_3 = 20;
float IR_VAL_4 = 4096;
int IR_THRESH_4 = 20;

// --- Motor & Servo Globals ---
const float TPM = 1873.0;  // Ticks per meter (Encoder calibration)
Motors motors;
int cam_current = CAM_SERVO_CENTER_US;
bool cam_control = false; // Flag to enable/disable camera servo updates

// --- Movement State ---
float spdMult = 0.75;
int srlToSpd = 1000; // Max speed scaling factor
int spd = 0;
int str_angle = SERVO_CENTER_US;  // Current steering angle
long lastEncoderPos = 0;
long currentEncoderPos = 0;
float headingPrev = 0.0;
float Current_angle = 0.0;

// --- Odometry & Navigation ---
float posX = 0.0, posY = 0.0;
float sectionHeading = 0.0;
// Parking distances received from Pi (Left, Front-Left, Front, Front-Right, Right)
int dstL = 0, dstFL = 0, dstF = 0, dstFR = 0, dstR = 0;

// Parking Logic Parameters
float moveSideDist = 35.0; // Distance to move sideways for alignment
float parkFrontDist[] = {85, 150}; // Target distances for parking spots (Front/Back)
bool park_is_back = false; // True if parking in the second (further) spot
bool turning = false;
int turnCount = 0;

// --- System State ---
long LEDtmr = 0;
long srlTmr = 0;
long stopTmr = 0;
bool stop_bot = 0;
bool running = false;
String command = "none"; // Current high-level state/command

// --- Helper Functions ---

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
int key = 0; // Holds the command key (first byte of serial packet)

long lastOdomSent = 0;
#define odomSendInterval 50 // Send odometry every 50ms

float lerp(float a, float b, float t){
    return a + (b-a)*t;
}

/**
 * Serial Communication Handler.
 * Reads bytes from the Raspberry Pi and updates robot state.
 * Protocol: [Key Byte] -> [Value Byte]
 */
void srl() {
    // Send Odometry to Pi periodically
    if (running && millis() - lastOdomSent > odomSendInterval) {
        // Format: [x, y, theta] (Note: Coordinate system adjustments applied here)
        COMM_SER.printf("[%f,%f,%f]\n", -posY, posX, -heading);
        lastOdomSent = millis();
    }

    while (COMM_SER.available()) {
        int v = COMM_SER.read();
        
        // Reset LED timer on activity
        if(command == "none"){
            LEDtmr = millis();
        }

        // --- Single Byte Commands/Keys (0-49) ---
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
        // Parking Commands
        else if (v == 6) {
            setCommand("moveRight","srl:6"); // Park Right Spot
            park_is_back = false;
            motors.hardbreak_enabled(false);
        } else if (v == 8) {
            setCommand("moveLeft","srl:8");  // Park Left Spot
            park_is_back = true;
            motors.hardbreak_enabled(false);
        } 
        
        // System Commands
        else if (v == 30){
            bnoResetNormalize(); // Reset IMU
        }
        else if (v == 47){
            LEDtmr = millis(); // Keep-alive
        }
        else if (v == 49) {
            tone(BUZZER_PIN, BUZZ_MID, 250); // Camera Handshake OK
        }
        // Store Key for next byte
        else if (v < 50) {
            key = v;
        }

        // --- Two Byte Commands (Key stored, Value >= 50) ---
        if (key != 0 && v >= 50) {
            // Throttle (Key 15)
            if (key == 15) {
                spd = map(v, 50, 250, -srlToSpd, srlToSpd);
            }
            // Steering (Key 16)
            if (key == 16) {
                str_angle = map(v, 50, 250, SERVO_MIN_US, SERVO_MAX_US);
            }
            // Camera Servo (Key 17)
            if (key == 17) {
                int ang = constrain(v - 50, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX);
                int t_us = map(ang, CAM_SERVO_ANGLE_MIN, CAM_SERVO_ANGLE_MAX, CAM_SERVO_MIN_US, CAM_SERVO_MAX_US);
                if(cam_control){
                    motors.setCamServoUs(t_us);
                }
                else{
                    motors.setCamServoUs(CAM_SERVO_CENTER_US);
                }
            }

            // Lidar distances from 5 directions (Keys 31-35)
            if(key == 31) dstL = v - 50;
            if(key == 32) dstFL = v - 50;
            if(key == 33) dstF = v - 50;
            if(key == 34) dstFR = v - 50;
            if(key == 35) dstR = v - 50;
            
            key = 0; // Reset key after processing value
        }
    }
}

/**
 * Odometry Calculation.
 * Updates global X, Y position based on encoder ticks and IMU heading.
 */
void odometry() {
    if(IR_Enabled){
        IR_VAL_3 = lerp(IR_VAL_3, analogRead(IR3_PIN), 0.1);
        IR_VAL_4 = lerp(IR_VAL_4, analogRead(IR4_PIN), 0.1);
    }
    currentEncoderPos = motors.getEncoderCount();
    float deltaEncoderPos = (currentEncoderPos - lastEncoderPos) / TPM;
    float ang = (heading + headingPrev) / 2; // Average heading during step
    headingPrev = heading;
    lastEncoderPos = currentEncoderPos;

    if (abs(deltaEncoderPos) > 0) {
        float deltaX = deltaEncoderPos * cos(ang * DEG_TO_RAD);
        float deltaY = deltaEncoderPos * sin(ang * DEG_TO_RAD);

        posX += deltaX;
        posY += deltaY;
    }
}

/**
 * FreeRTOS Task: Serial & Odometry Loop.
 * Runs on Core 1 to handle communication independently of motor control.
 */
void srlRead(void* pvParameters) {
    while (true) {
        odometry();
        srl();

        // Blink Status LED based on activity
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

// --- Setup ---
void setup() {
    // Initialize Sensors & Comms
    initBNO();
    startBNOCalcTask();
    initADC();

    Serial1.begin(115200, SERIAL_8N1, 19, 23); // To Pi
    Serial.begin(115200); // Debug
    enableIR();
    initOLED();
    startOLEDDisplayTask();
    displayText("Wait...");

    motors.begin();
    initIO();

    // Clear serial buffer
    while (COMM_SER.available() > 0) {
        COMM_SER.read();
    }

    startSerialReadTask();  // Start comms task
    motors.setServoUs(SERVO_CENTER_US);
    motors.setCamServoUs(CAM_SERVO_CENTER_US);
    
    // Calibrate IMU offset
    bnoCalcOffset(1500);
    
    displayText("All okay!");
    // Startup Beep
    tone(BUZZER_PIN, BUZZ_HIGH, 250);
    tone(BUZZER_PIN, BUZZ_MID, 250);
    noTone(BUZZER_PIN);

    // Handshake with Pi camera system
    COMM_SER.println(F("CONF_CAM"));

    // Wait for Start Button Press
    while (digitalRead(BUTTON_PIN) == HIGH) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    disableIR();
    parkingSpeedHigh();
    
    // Wait for Button Release
    while (digitalRead(BUTTON_PIN) == LOW) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    // Start Sequence
    bnoCalcOffset(200);    // Zero heading
    cam_control = true;
    tone(BUZZER_PIN, BUZZ_HIGH, 500);
    
    COMM_SER.println(F("Boot")); // Signal Pi to start
    displayText("");
    
    // Reset State
    motors.setServoUs(SERVO_CENTER_US);
    motors.setMotorSpeed(0);
    lastEncoderPos = motors.getEncoderCount();
    headingPrev = heading;
    posX = 0.0;
    posY = 0.0;

    sectionHeading = heading;
    startTime = millis();
    running = true;

    // Hardware Test Mode (Optional)
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

/**
 * Complex Parking Maneuver.
 * Executes a series of turns to wiggle into the parking box.
 * @param dir: 1 for Left, -1 for Right
 */
void paark(int dir){
    cam_control = false;

    motors.setServoUs(SERVO_CENTER_US);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    move_pos(0.15, 0.0); // Move forward slightly

    // Wiggle maneuver
    turn_angle_opp(25*dir);
    turn_angle(25*dir);
    turn_angle_opp(50*dir);
    turn_angle(65*dir);
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

// --- Main Loop ---
void loop() {
    displayText(command);

    // --- Initialization Turns ---
    if (command == "right45") {
        turn_angle(45); // Actually turns 90 deg relative to start?
        setCommand("none","loop:right45");
        COMM_SER.println("Turned");
    }

    if (command == "left45") {
        turn_angle(-45);
        setCommand("none","loop:left45");
        COMM_SER.println("Turned");
    }

    // --- Parking Setup Moves ---
    if (command == "straight") {
        motors.setServoUs(SERVO_CENTER_US);
        move_pos(0.10);
        parkingSpeedLow();
        turn_angle(0); // Re-align to 0
        setCommand("none","loop:straight");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
        motors.hardbreak_enabled(true);
    }
    if (command == "passTurn") {
        // Adjust heading if facing wrong way
        if (heading > 0) {
            turn_angle(135);
        } else {
            turn_angle(-135);
        }
        parkingSpeedLow();
        turn_angle(0);
        setCommand("none","loop:passTurn");
        COMM_SER.println("Done");
        posX = 0.0;
        posY = 0.0;
        motors.hardbreak_enabled(true);
    }

    // --- Main Driving Mode ---
    if (command == "go") {
        motors.setMotorSpeed(spd);
        motors.setServoUs(str_angle);
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield
    }

    // --- Parking Sequence Start ---
    if(command == "moveRight"){
        // Align 45 degrees to approach parking lane
        turn_angle(0);
        turn_angle(45);
        motors.setServoUs(SERVO_CENTER_US);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        // Calculate diagonal distance
        float tomove = dstFR - moveSideDist;
        tomove = tomove * 1.4142; // Hypotenuse
        
        tone(BUZZER_PIN, BUZZ_LOW, 200);
        move_pos(tomove/100);
        tone(BUZZER_PIN, BUZZ_HIGH, 200);
        
        setCommand("parkRight","loop:moveRight");
    }
    if(command == "moveLeft"){
        turn_angle(0);
        turn_angle(-45);
        motors.setServoUs(SERVO_CENTER_US);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        float tomove = dstFL - moveSideDist;
        tomove = tomove * 1.4142;
        
        tone(BUZZER_PIN, BUZZ_LOW, 200);
        move_pos(tomove/100);
        tone(BUZZER_PIN, BUZZ_HIGH, 200);
        
        setCommand("parkLeft","loop:moveLeft");
    }

    // --- Parking Execution (Left Side) ---
    if(command == "parkLeft"){
        displayText("Parking Left");
        turn_angle(0); // Face forward
        vTaskDelay(500 / portTICK_PERIOD_MS);
        motors.setServoUs(SERVO_CENTER_US);

        // Move forward in steps to reach target distance
        parkingSpeedHigh();
        float toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        parkingSpeedLow();
        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        toMove = dstF - parkFrontDist[park_is_back ? 1 : 0];
        move_pos(toMove/100, 0.0);

        displayText("Initializing Parking");

        turn_angle_opp(90);
        turn_angle_opp(90);
        move_pos(-0.27, 90.0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        // Decide final wiggle direction based on side clearance
        if(dstL > dstR){
            displayText("park_fin_R");
            setCommand("park_fin_R","loop:parkLeft");
        } else {
            displayText("park_fin_L");
            setCommand("park_fin_L","loop:parkLeft");
        }
    }

    // --- Parking Execution (Right Side) ---
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
        move_pos(-0.27, -90.0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(dstL > dstR){
            displayText("park_fin_R");
            setCommand("park_fin_R","loop:parkRight");
        } else {
            displayText("park_fin_L");
            setCommand("park_fin_L","loop:parkRight");
        }
    }

    // --- Final Parking Wiggle ---
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

    // --- Idle State ---
    if (command == "none") {
        motors.setMotorSpeed(0);
        motors.setServoUs(SERVO_CENTER_US);
    }
}
