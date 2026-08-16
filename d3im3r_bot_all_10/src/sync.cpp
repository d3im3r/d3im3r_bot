#include "sync.h"
#include "app_config.h"

#include <math.h>

static bool g_heading_locked = false;
static float g_yaw_ref_rad = 0.0f;

static inline float clamp_value(float x, float limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

static inline float wrap_to_pi(float angle)
{
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s)
{
    if (!SYNC_ENABLE) return false;
    if (fabs(v_cmd_m_s) < SYNC_MIN_LINEAR_CMD_M_S) return false;
    if (fabs(w_cmd_rad_s) >= SYNC_ANGULAR_EPS_RAD_S) return false;
    return true;
}

void sync_reset()
{
    g_heading_locked = false;
    g_yaw_ref_rad = 0.0f;
}

SyncOutput sync_apply_hybrid_straight_correction(
    float left_ref_rad_s,
    float right_ref_rad_s,
    float left_meas_rad_s,
    float right_meas_rad_s,
    float v_cmd_m_s,
    float w_cmd_rad_s,
    float yaw_now_rad,
    bool imu_available
)
{
    SyncOutput out;
    out.left_rad_s = left_ref_rad_s;
    out.right_rad_s = right_ref_rad_s;
    out.active = false;
    out.encoder_error_rad_s = 0.0f;
    out.encoder_corr_rad_s = 0.0f;
    out.yaw_error_rad = 0.0f;
    out.imu_corr_rad_s = 0.0f;
    out.total_corr_rad_s = 0.0f;

    if (!sync_should_activate(v_cmd_m_s, w_cmd_rad_s)) {
        sync_reset();
        return out;
    }

    out.active = true;

    // 1. Bloqueo de rumbo con IMU
    if (imu_available && SYNC_IMU_ENABLE) {
        if (!g_heading_locked) {
            g_yaw_ref_rad = yaw_now_rad;
            g_heading_locked = true;
        }
    }

    // 2. Corrección por encoders
    if (SYNC_ENC_ENABLE) {
        const float error_enc = left_meas_rad_s - right_meas_rad_s;
        float corr_enc = clamp_value(SYNC_ENC_KP * error_enc, SYNC_ENC_MAX_CORR_RAD_S);
        out.encoder_error_rad_s = error_enc;
        out.encoder_corr_rad_s = corr_enc;
    }

    // 3. Corrección por IMU
    if (imu_available && SYNC_IMU_ENABLE && g_heading_locked) {
        float yaw_error = wrap_to_pi(g_yaw_ref_rad - yaw_now_rad);
        if (fabs(yaw_error) < SYNC_YAW_DEADBAND_RAD) {
            yaw_error = 0.0f;
        }
        float corr_imu = clamp_value(SYNC_IMU_KP * yaw_error, SYNC_IMU_MAX_CORR_RAD_S);
        out.yaw_error_rad = yaw_error;
        out.imu_corr_rad_s = corr_imu;
    }

    // 4. Corrección total
    float total_corr = clamp_value(out.encoder_corr_rad_s + out.imu_corr_rad_s, SYNC_TOTAL_MAX_CORR_RAD_S);
    out.total_corr_rad_s = total_corr;

    // 5. Aplicación sobre referencias finales
    out.left_rad_s = kinematics_clamp_wheel_ref(left_ref_rad_s - total_corr);
    out.right_rad_s = kinematics_clamp_wheel_ref(right_ref_rad_s + total_corr);

    return out;
}