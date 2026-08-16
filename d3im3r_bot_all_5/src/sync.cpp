#include "sync.h"
#include "app_config.h"

#include <math.h>

static float clamp_value(float x, float limit_abs)
{
    if (x > limit_abs) {
        return limit_abs;
    }

    if (x < -limit_abs) {
        return -limit_abs;
    }

    return x;
}

bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s)
{
    if (!SYNC_ENABLE) {
        return false;
    }

    if (fabs(v_cmd_m_s) < SYNC_MIN_LINEAR_CMD_M_S) {
        return false;
    }

    if (fabs(w_cmd_rad_s) >= SYNC_ANGULAR_EPS_RAD_S) {
        return false;
    }

    return true;
}

SyncOutput sync_apply_straight_correction(
    float left_ref_rad_s,
    float right_ref_rad_s,
    float left_meas_rad_s,
    float right_meas_rad_s,
    float v_cmd_m_s,
    float w_cmd_rad_s
)
{
    SyncOutput out;

    out.left_rad_s = left_ref_rad_s;
    out.right_rad_s = right_ref_rad_s;
    out.active = false;
    out.error_rad_s = 0.0f;
    out.correction_rad_s = 0.0f;

    if (!sync_should_activate(v_cmd_m_s, w_cmd_rad_s)) {
        return out;
    }

    // Error de sincronía:
    // positivo si la rueda izquierda va más rápido que la derecha
    const float sync_error = left_meas_rad_s - right_meas_rad_s;

    float corr = SYNC_KP * sync_error;
    corr = clamp_value(corr, SYNC_MAX_CORR_RAD_S);

    out.left_rad_s = left_ref_rad_s - corr;
    out.right_rad_s = right_ref_rad_s + corr;

    out.left_rad_s = kinematics_clamp_wheel_ref(out.left_rad_s);
    out.right_rad_s = kinematics_clamp_wheel_ref(out.right_rad_s);

    out.active = true;
    out.error_rad_s = sync_error;
    out.correction_rad_s = corr;

    return out;
}