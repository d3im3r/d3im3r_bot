#pragma once

#include <Arduino.h>

// ===== IMU =====

extern float imu_qx;
extern float imu_qy;
extern float imu_qz;
extern float imu_qw;

extern uint16_t calibration_status;

// ===== TOF =====

extern float tof_distances[3];

// ===== API =====

bool sensors_init();
void sensors_update();