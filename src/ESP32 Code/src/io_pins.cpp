#include "io_pins.h"

void initIO() {
    // Configure pin modes
    pinMode(STATUS_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(IR1_PIN, INPUT);
    pinMode(IR2_PIN, INPUT);
    pinMode(IR3_PIN, INPUT);
    pinMode(IR4_PIN, INPUT);

    // Initial status indication
    digitalWrite(STATUS_LED, HIGH);
}
