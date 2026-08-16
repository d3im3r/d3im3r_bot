#ifndef SYNC_H
#define SYNC_H

#include <Arduino.h>
#include "kinematics.h"

typedef struct
{
    float left_rad_s;
    float right_rad_s;
    bool active;
    float error_rad_s;
    float correction_rad_s;
} SyncOutput;

bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s);

SyncOutput sync_apply_straight_correction(
    float left_ref_rad_s,
    float right_ref_rad_s,
    float left_meas_rad_s,
    float right_meas_rad_s,
    float v_cmd_m_s,
    float w_cmd_rad_s
);

#endif