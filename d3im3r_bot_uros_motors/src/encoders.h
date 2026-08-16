#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <stdint.h>

bool encoders_init();

int32_t encoders_get_left_ticks();
int32_t encoders_get_right_ticks();

void encoders_set_left_ticks(int32_t value);
void encoders_set_right_ticks(int32_t value);

void encoders_reset();

#endif