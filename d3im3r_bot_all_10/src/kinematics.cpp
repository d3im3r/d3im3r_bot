#include "kinematics.h"
#include "app_config.h"
#include <math.h>

WheelSpeedRefs kinematics_inverse(float v_m_s, float w_rad_s)
{
    WheelSpeedRefs refs;
    const float half_base = WHEEL_BASE_M * 0.5f;

    refs.left_rad_s  = (v_m_s - (half_base * w_rad_s)) / WHEEL_RADIUS_M;
    refs.right_rad_s = (v_m_s + (half_base * w_rad_s)) / WHEEL_RADIUS_M;

    kinematics_clamp_wheel_refs(&refs);
    return refs;
}

WheelSpeedRefs kinematics_vw_to_wheels(float v_m_s, float w_rad_s)
{
    return kinematics_inverse(v_m_s, w_rad_s);
}

BodyTwist2D kinematics_forward(float left_rad_s, float right_rad_s)
{
    BodyTwist2D twist;
    twist.v_m_s   = (WHEEL_RADIUS_M * 0.5f) * (right_rad_s + left_rad_s);
    twist.w_rad_s = (WHEEL_RADIUS_M / WHEEL_BASE_M) * (right_rad_s - left_rad_s);
    return twist;
}

float kinematics_clamp_wheel_ref(float wheel_rad_s)
{
    if (wheel_rad_s > WHEEL_MAX_RAD_S)  return WHEEL_MAX_RAD_S;
    if (wheel_rad_s < -WHEEL_MAX_RAD_S) return -WHEEL_MAX_RAD_S;
    return wheel_rad_s;
}

void kinematics_clamp_wheel_refs(WheelSpeedRefs* refs)
{
    if (refs != nullptr) {
        refs->left_rad_s  = kinematics_clamp_wheel_ref(refs->left_rad_s);
        refs->right_rad_s = kinematics_clamp_wheel_ref(refs->right_rad_s);
    }
}