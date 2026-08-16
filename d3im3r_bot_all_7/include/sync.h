#ifndef SYNC_H
#define SYNC_H

#include <Arduino.h>
#include "kinematics.h"

typedef struct
{
    float left_rad_s;
    float right_rad_s;

    bool active;

    float encoder_error_rad_s;
    float encoder_corr_rad_s;

    float yaw_error_rad;
    float imu_corr_rad_s;

    float total_corr_rad_s;
} SyncOutput;

bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s);

SyncOutput sync_apply_hybrid_straight_correction(
    float left_ref_rad_s,
    float right_ref_rad_s,
    float left_meas_rad_s,
    float right_meas_rad_s,
    float v_cmd_m_s,
    float w_cmd_rad_s,
    float yaw_now_rad
);

void sync_reset();

#endif