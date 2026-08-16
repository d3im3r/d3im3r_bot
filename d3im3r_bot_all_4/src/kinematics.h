#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

typedef struct
{
    float left_rad_s;
    float right_rad_s;
} WheelSpeedRefs;

WheelSpeedRefs kinematics_vw_to_wheels(float v_m_s, float w_rad_s);

float kinematics_clamp_wheel_ref(float wheel_rad_s);
void kinematics_clamp_wheel_refs(WheelSpeedRefs* refs);

#endif