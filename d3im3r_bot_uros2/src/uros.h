#pragma once

#include <Arduino.h>

extern bool uros_transport_ok;
extern bool uros_support_ok;
extern bool uros_node_ok;
extern bool uros_publishers_ok;

bool uros_init();

void uros_publish_imu(float x, float y, float z, float w);
void uros_publish_calibration(uint16_t status);
void uros_publish_tof(float d1, float d2, float d3);

void uros_spin();