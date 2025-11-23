#include <Arduino.h>

void setUseIR(bool stt);

void turn_angle(float target_angle);
void turn_angle_opp(float target_angle);

void move_pos_func(float distance, float trg_ang);
void move_pos(float distance);
void move_pos(float distance, float target_heading);

void head_into(float trg);

void parkingSpeedHigh();
void parkingSpeedLow();