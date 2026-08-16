#pragma once

#include <Arduino.h>

// Quaternion
extern float imu_qx;
extern float imu_qy;
extern float imu_qz;
extern float imu_qw;

// Yaw en rad
extern float imu_yaw;

// Calibración compacta
extern uint16_t calibration_status;

// Calibración separada para OLED/debug
extern uint8_t cal_sys;
extern uint8_t cal_gyro;
extern uint8_t cal_accel;
extern uint8_t cal_mag;

// Distancias [m]
// 0=center, 1=left, 2=right
extern float tof_distances[3];

// Estados
extern bool bno_ok;
extern bool tof_center_ok;
extern bool tof_left_ok;
extern bool tof_right_ok;

bool sensors_init();

void sensors_update_imu();

void sensors_update_tof();