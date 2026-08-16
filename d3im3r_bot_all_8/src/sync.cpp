#include "sync.h"
#include "app_config.h"

#include <math.h>

// =====================================================
// Estado interno de sincronía
// =====================================================
static bool g_heading_locked = false;
static float g_yaw_ref_rad = 0.0f;

// =====================================================
// Utilidades
// =====================================================
static float clamp_value(float x, float limit)
{
    if (x > limit) {
        return limit;
    }

    if (x < -limit) {
        return -limit;
    }

    return x;
}

static float wrap_to_pi(float angle)
{
    while (angle > PI) {
        angle -= 2.0f * PI;
    }

    while (angle < -PI) {
        angle += 2.0f * PI;
    }

    return angle;
}

// =====================================================
// Activación
// =====================================================
bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s)
{
    if (!SYNC_ENABLE) {
        return false;
    }

    // Si no hay avance lineal suficiente, no tiene sentido
    // bloquear rumbo.
    if (fabs(v_cmd_m_s) < SYNC_MIN_LINEAR_CMD_M_S) {
        return false;
    }

    // Si el usuario o ROS 2 está pidiendo giro, no corregimos
    // línea recta, porque el robot sí debe cambiar su yaw.
    if (fabs(w_cmd_rad_s) >= SYNC_ANGULAR_EPS_RAD_S) {
        return false;
    }

    return true;
}

// =====================================================
// Reset
// =====================================================
void sync_reset()
{
    g_heading_locked = false;
    g_yaw_ref_rad = 0.0f;
}

// =====================================================
// Core de sincronía híbrida
// =====================================================
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

    // Valores por defecto: sin corrección
    out.left_rad_s = left_ref_rad_s;
    out.right_rad_s = right_ref_rad_s;

    out.active = false;

    out.encoder_error_rad_s = 0.0f;
    out.encoder_corr_rad_s = 0.0f;

    out.yaw_error_rad = 0.0f;
    out.imu_corr_rad_s = 0.0f;

    out.total_corr_rad_s = 0.0f;

    // Verificar si debe activarse la corrección
    const bool active = sync_should_activate(v_cmd_m_s, w_cmd_rad_s);

    if (!active) {
        sync_reset();
        return out;
    }

    out.active = true;

    // =================================================
    // 1. Bloqueo de rumbo con IMU
    // =================================================
    //
    // Al iniciar un tramo recto, se guarda el yaw actual
    // como referencia. Luego se compara el yaw actual contra
    // ese valor.
    //
    // Importante:
    // Este bloqueo solo se usa si la IMU está disponible.
    //
    // =================================================
    if (imu_available && SYNC_IMU_ENABLE) {
        if (!g_heading_locked) {
            g_yaw_ref_rad = yaw_now_rad;
            g_heading_locked = true;
        }
    }

    // =================================================
    // 2. Corrección por encoders
    // =================================================
    //
    // error_enc = w_left_meas - w_right_meas
    //
    // Si la rueda izquierda mide menos que la derecha:
    // error_enc < 0
    // total_corr < 0
    //
    // Con la convención final:
    // left_ref  = left_ref  - total_corr
    // right_ref = right_ref + total_corr
    //
    // entonces la izquierda sube y la derecha baja.
    //
    // =================================================
    if (SYNC_ENC_ENABLE) {
        const float error_enc = left_meas_rad_s - right_meas_rad_s;

        float corr_enc = SYNC_ENC_KP * error_enc;
        corr_enc = clamp_value(corr_enc, SYNC_ENC_MAX_CORR_RAD_S);

        out.encoder_error_rad_s = error_enc;
        out.encoder_corr_rad_s = corr_enc;
    }

    // =================================================
    // 3. Corrección por IMU
    // =================================================
    //
    // yaw_error = yaw_ref - yaw_actual
    //
    // Si el robot se desvía hacia la izquierda y el yaw
    // positivo corresponde a giro antihorario, yaw_actual
    // aumenta, por tanto:
    //
    // yaw_error < 0
    // corr_imu < 0
    //
    // Con la convención final:
    //
    // left_ref  = left_ref  - total_corr
    // right_ref = right_ref + total_corr
    //
    // la rueda izquierda aumenta y la derecha disminuye,
    // ayudando a corregir el desvío.
    //
    // =================================================
    if (imu_available && SYNC_IMU_ENABLE && g_heading_locked) {
        float yaw_error = wrap_to_pi(g_yaw_ref_rad - yaw_now_rad);

        // Zona muerta para evitar microcorrecciones innecesarias
        if (fabs(yaw_error) < SYNC_YAW_DEADBAND_RAD) {
            yaw_error = 0.0f;
        }

        float corr_imu = SYNC_IMU_KP * yaw_error;
        corr_imu = clamp_value(corr_imu, SYNC_IMU_MAX_CORR_RAD_S);

        out.yaw_error_rad = yaw_error;
        out.imu_corr_rad_s = corr_imu;
    }

    // =================================================
    // 4. Corrección total
    // =====================================================
    float total_corr = out.encoder_corr_rad_s + out.imu_corr_rad_s;
    total_corr = clamp_value(total_corr, SYNC_TOTAL_MAX_CORR_RAD_S);

    out.total_corr_rad_s = total_corr;

    // =================================================
    // 5. Aplicación sobre referencias
    // =====================================================
    //
    // Esta convención es coherente con:
    //
    // error_enc = left_meas - right_meas
    //
    // y con yaw_error = yaw_ref - yaw_actual,
    // asumiendo yaw positivo antihorario.
    //
    // Si notas que la corrección empeora el desvío,
    // invierte únicamente estos dos signos:
    //
    // out.left_rad_s  = left_ref_rad_s  + total_corr;
    // out.right_rad_s = right_ref_rad_s - total_corr;
    //
    // =================================================
    out.left_rad_s = left_ref_rad_s - total_corr;
    out.right_rad_s = right_ref_rad_s + total_corr;

    // Clamp final para proteger referencias
    out.left_rad_s = kinematics_clamp_wheel_ref(out.left_rad_s);
    out.right_rad_s = kinematics_clamp_wheel_ref(out.right_rad_s);

    return out;
}