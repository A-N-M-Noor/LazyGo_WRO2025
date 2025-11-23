#include "OLED.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include "battery.h"
#include "bno.h"
#include "motors.h"

// Declare global posX and posY variables defined elsewhere
extern float posX;
extern float posY;
extern Motors motors;
extern String command;

extern float IR_VAL_3;
extern float IR_VAL_4;

// Declare global distance sensor variables defined elsewhere
// extern int distLeft;
// extern int distRight;
// extern int distFront;
extern int turnCount;
extern bool stop_bot;
extern bool running;

String disp;

unsigned int seconds = 0;

// OLED object (128x64 SSD1306, I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void initOLED() {
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "Wait...");
    u8g2.sendBuffer();
}

void displayText(String txt) {
    disp = txt;
}

void displayTask(void* pvParameters) {
    unsigned long lastDisplayUpdate = 0;
    while (1) {
        unsigned long currentTime = millis();
        if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
            updateBatteryVoltage();
            if (running) {
                seconds = (millis() - startTime) / 1000;
            }
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_4x6_tf);
            u8g2.drawStr(0, 10, disp.c_str());

            char cmdBuf[16];
            snprintf(cmdBuf, sizeof(cmdBuf), "%s", command);
            int cmdWdth = u8g2.getStrWidth(cmdBuf);
            u8g2.drawStr((128 - cmdWdth), 10, cmdBuf);

            char timeBuf[9];
            snprintf(timeBuf, sizeof(timeBuf), "%d:%d", seconds / 60, seconds % 60);
            int timeWidth = u8g2.getStrWidth(timeBuf);
            u8g2.drawStr((128 - timeWidth), 25, timeBuf);

            u8g2.setFont(u8g2_font_ncenB10_tr);
            char headingBuf[16];
            snprintf(headingBuf, sizeof(headingBuf), "%.2f", heading);
            int headingWidth = u8g2.getStrWidth(headingBuf);
            u8g2.drawStr((128 - headingWidth) / 2, 20, headingBuf);

            u8g2.setFont(u8g2_font_4x6_tf);
            char _headingBuf[16];
            snprintf(_headingBuf, sizeof(_headingBuf), "%.2f", heading_norm);
            headingWidth = u8g2.getStrWidth(_headingBuf);
            u8g2.drawStr((128 - headingWidth), 60, _headingBuf);

            u8g2.setFont(u8g2_font_ncenB10_tr);
            if (disp == "All okay!") {
                char encBuf[20];
                snprintf(encBuf, sizeof(encBuf), "%d", motors.getEncoderCount());
                int encWidth = u8g2.getStrWidth(encBuf);
                u8g2.drawStr((128 - encWidth) / 2, 40, encBuf);
            } else {
                char posBuf[20];
                snprintf(posBuf, sizeof(posBuf), "[%.2f,%.2f]", posX, posY);
                int posWidth = u8g2.getStrWidth(posBuf);
                u8g2.drawStr((128 - posWidth) / 2, 40, posBuf);
            }

            // char distBuf[15];
            // snprintf(distBuf, sizeof(distBuf), "%d|%d|%d", distLeft, distFront, distRight);
            // int distWidth = u8g2.getStrWidth(distBuf);
            // u8g2.drawStr((128 - distWidth) / 2, 60, distBuf);

            // Battery voltage (bottom-left), small font, two decimals
            u8g2.setFont(u8g2_font_4x6_tf);
            char batBuf[16];
            snprintf(batBuf, sizeof(batBuf), "B:%.2fV", getBatteryVoltage());
            u8g2.drawStr(0, 60, batBuf);

            // Low voltage warning (bottom-right) when batteryLow is true
            if (batteryLow) {
                const char* warn = "LOW VOLT";
                int warnWidth = u8g2.getStrWidth(warn);
                // ----------------UPDATE PRINT LOCATION AFTER TESTING:------------------
                u8g2.drawStr(128 - warnWidth, 60, warn);
            }

            u8g2.sendBuffer();
            lastDisplayUpdate = currentTime;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield frequently
    }
}

void startOLEDDisplayTask() {
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 5, NULL, 1);  // Core 1, low priority
}