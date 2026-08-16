#pragma once

#include <Arduino.h>

bool uros_init();

void uros_publish_orientation(float qx,float qy,float qz,float qw);

void uros_publish_yaw(float yaw);

void uros_publish_calibration(uint16_t status);

void uros_publish_tof(float c,float l,float r);

void uros_spin();