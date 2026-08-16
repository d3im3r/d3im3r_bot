#pragma once

#include <Arduino.h>

// ===== Estado sensores =====
extern bool bno_ok;
extern bool tof_center_ok;
extern bool tof_left_ok;
extern bool tof_right_ok;

// ===== Datos IMU =====
extern float imu_qx;
extern float imu_qy;
extern float imu_qz;
extern float imu_qw;

extern uint16_t calibration_status;

// ===== Datos TOF =====
// x = central, y = izquierdo, z = derecho
extern float tof_distances[3];

// ===== API =====
bool sensors_init();
void sensors_update();