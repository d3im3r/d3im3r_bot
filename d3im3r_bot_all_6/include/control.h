#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

bool control_init();
void control_update();

void control_set_left_ref(float ref_rad_s);
void control_set_right_ref(float ref_rad_s);

float control_get_left_ref();
float control_get_right_ref();

float control_get_left_u();
float control_get_right_u();

float control_get_left_error();
float control_get_right_error();

#endif