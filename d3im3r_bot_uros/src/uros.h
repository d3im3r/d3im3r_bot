#pragma once

#include <Arduino.h>

void uros_init();

void uros_publish_imu(float x,float y,float z,float w);

void uros_publish_calibration(uint16_t status);

void uros_publish_tof(float d1,float d2,float d3);

void uros_spin();