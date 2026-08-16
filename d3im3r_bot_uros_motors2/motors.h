#pragma once

#include <Arduino.h>

void motors_init();
void motors_update();

void motors_set_left(float cmd_percent);
void motors_set_right(float cmd_percent);

float motors_get_left_applied();
float motors_get_right_applied();

void motors_stop_all();