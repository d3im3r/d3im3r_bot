#include "kinematics.h"
#include "app_config.h"

#include <math.h>

WheelSpeedRefs kinematics_vw_to_wheels(float v_m_s, float w_rad_s)
{
    WheelSpeedRefs refs;

    const float half_base = WHEEL_BASE_M * 0.5f;

    refs.left_rad_s  = (v_m_s - (half_base * w_rad_s)) / WHEEL_RADIUS_M;
    refs.right_rad_s = (v_m_s + (half_base * w_rad_s)) / WHEEL_RADIUS_M;

    kinematics_clamp_wheel_refs(&refs);

    return refs;
}

float kinematics_clamp_wheel_ref(float wheel_rad_s)
{
    if (wheel_rad_s > WHEEL_MAX_RAD_S) {
        return WHEEL_MAX_RAD_S;
    }

    if (wheel_rad_s < -WHEEL_MAX_RAD_S) {
        return -WHEEL_MAX_RAD_S;
    }

    return wheel_rad_s;
}

void kinematics_clamp_wheel_refs(WheelSpeedRefs* refs)
{
    if (refs == nullptr) {
        return;
    }

    refs->left_rad_s  = kinematics_clamp_wheel_ref(refs->left_rad_s);
    refs->right_rad_s = kinematics_clamp_wheel_ref(refs->right_rad_s);
}