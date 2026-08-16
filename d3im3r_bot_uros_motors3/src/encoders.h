#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <stdint.h>

bool encoders_init();
void encoders_update();

int32_t encoders_get_left_ticks();
int32_t encoders_get_right_ticks();

void encoders_set_left_ticks(int32_t value);
void encoders_set_right_ticks(int32_t value);
void encoders_reset();

// Velocidades izquierda
float encoders_get_left_rad_s();
float encoders_get_left_rpm();
float encoders_get_left_m_s();

// Velocidades derecha
float encoders_get_right_rad_s();
float encoders_get_right_rpm();
float encoders_get_right_m_s();

#endif