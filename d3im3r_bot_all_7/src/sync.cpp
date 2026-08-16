#include "sync.h"
#include "app_config.h"

#include <math.h>

// =========================
// Estado interno
// =========================
static bool g_heading_locked = false;
static float g_yaw_ref = 0.0f;

// =========================
// Utils
// =========================
static float clamp_value(float x, float limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

static float wrap_to_pi(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// =========================
// Activación
// =========================
bool sync_should_activate(float v_cmd, float w_cmd)
{
    if (!SYNC_ENABLE) return false;

    if (fabs(v_cmd) < SYNC_MIN_LINEAR_CMD_M_S)
        return false;

    if (fabs(w_cmd) >= SYNC_ANGULAR_EPS_RAD_S)
        return false;

    return true;
}

// =========================
// Reset
// =========================
void sync_reset()
{
    g_heading_locked = false;
    g_yaw_ref = 0.0f;
}

// =========================
// Core híbrido
// =========================
SyncOutput sync_apply_hybrid_straight_correction(
    float left_ref,
    float right_ref,
    float left_meas,
    float right_meas,
    float v_cmd,
    float w_cmd,
    float yaw_now
)
{
    SyncOutput out;

    // Inicialización
    out.left_rad_s = left_ref;
    out.right_rad_s = right_ref;
    out.active = false;

    out.encoder_error_rad_s = 0.0f;
    out.encoder_corr_rad_s = 0.0f;

    out.yaw_error_rad = 0.0f;
    out.imu_corr_rad_s = 0.0f;

    out.total_corr_rad_s = 0.0f;

    // ¿Debe activarse?
    const bool active = sync_should_activate(v_cmd, w_cmd);

    if (!active)
    {
        sync_reset();
        return out;
    }

    out.active = true;

    // =========================
    // LOCK DE RUMBO
    // =========================
    if (!g_heading_locked)
    {
        g_yaw_ref = yaw_now;
        g_heading_locked = true;
    }

    // =========================
    // 1. CORRECCIÓN POR ENCODER (principal)
    // =========================
    if (SYNC_ENC_ENABLE)
    {
        float error_enc = left_meas - right_meas;

        float corr_enc = SYNC_ENC_KP * error_enc;
        corr_enc = clamp_value(corr_enc, SYNC_ENC_MAX_CORR_RAD_S);

        out.encoder_error_rad_s = error_enc;
        out.encoder_corr_rad_s = corr_enc;
    }

    // =========================
    // 2. CORRECCIÓN POR IMU (suave)
    // =========================
    if (SYNC_IMU_ENABLE)
    {
        float yaw_error = wrap_to_pi(g_yaw_ref - yaw_now);

        // Deadband (muy importante)
        const float yaw_deadband = 0.02f;//0.03f;  // ~1.7 grados
        if (fabs(yaw_error) < yaw_deadband)
        {
            yaw_error = 0.0f;
        }

        float corr_imu = SYNC_IMU_KP * yaw_error;
        corr_imu = clamp_value(corr_imu, SYNC_IMU_MAX_CORR_RAD_S);

        out.yaw_error_rad = yaw_error;
        out.imu_corr_rad_s = corr_imu;
    }

    // =========================
    // 3. CORRECCIÓN TOTAL
    // =========================
    float total_corr = out.encoder_corr_rad_s + out.imu_corr_rad_s;

    total_corr = clamp_value(total_corr, SYNC_TOTAL_MAX_CORR_RAD_S);

    out.total_corr_rad_s = total_corr;

    // ⚠️ IMPORTANTE: esta convención ya validada (NO invertir aquí)
    out.left_rad_s  = left_ref  - total_corr;
    out.right_rad_s = right_ref + total_corr;

    // Clamp final
    out.left_rad_s  = kinematics_clamp_wheel_ref(out.left_rad_s);
    out.right_rad_s = kinematics_clamp_wheel_ref(out.right_rad_s);

    return out;
}