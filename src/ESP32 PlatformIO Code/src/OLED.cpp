#include <Arduino.h>
#include <U8g2lib.h>
#include "OLED.h"
#include "battery.h"
#include "IMU.h"

// Declare global posX and posY variables defined elsewhere
extern float posX;
extern float posY;

// OLED object (128x64 SSD1306, I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void initOLED() {
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "MPU6050 Starting...");
    u8g2.sendBuffer();
}

void displayTask(void *pvParameters) {
    unsigned long lastDisplayUpdate = 0;
    while (1) {
        unsigned long currentTime = millis();
        if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
            // Serial.println("\ndisplay update\n");
            updateBatteryVoltage();
            float secondsSinceWake = (currentTime - startTime) / 1000.0;

            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_4x6_tf);
            char timeBuf[16];
            if (secondsSinceWake > 99) {
                snprintf(timeBuf, sizeof(timeBuf), "%.1f m", secondsSinceWake / 60.0);
            } else {
                snprintf(timeBuf, sizeof(timeBuf), "%.1f s", secondsSinceWake);
            }
            u8g2.drawStr(0, 10, timeBuf);

            char voltBuf[16];
            snprintf(voltBuf, sizeof(voltBuf), "%.2f V", getBatteryVoltage());
            u8g2.drawStr(0, 18, voltBuf);

            char rateBuf[16];
            snprintf(rateBuf, sizeof(timeBuf), "%.1f Hz", pollingRate);
            int rateWidth = u8g2.getStrWidth(rateBuf);
            u8g2.drawStr(128 - rateWidth, 10, rateBuf);

            u8g2.setFont(u8g2_font_ncenB14_tr);
            char axisBuf[16];
            snprintf(axisBuf, sizeof(axisBuf), "%s: %.2f°", "Yaw", continuousYaw);
            int axisWidth = u8g2.getStrWidth(axisBuf);
            u8g2.drawStr((128 - axisWidth) / 2, 30, axisBuf);

            // Display posX and posY, same font and style as Yaw
            char posXBuf[16];
            snprintf(posXBuf, sizeof(posXBuf), "X: %.2f", posX);
            int posXWidth = u8g2.getStrWidth(posXBuf);
            u8g2.drawStr((128 - posXWidth) / 2, 50, posXBuf);

            char posYBuf[16];
            snprintf(posYBuf, sizeof(posYBuf), "Y: %.2f", posY);
            int posYWidth = u8g2.getStrWidth(posYBuf);
            u8g2.drawStr((128 - posYWidth) / 2, 64, posYBuf);

            u8g2.sendBuffer();
            lastDisplayUpdate = currentTime;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield frequently
    }
}

void startOLEDDisplayTask() {
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 5, NULL, 1); // Core 1, low priority
}