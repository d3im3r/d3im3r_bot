#include "safety.h"
#include "app_config.h"

#include <math.h>

// =====================================================
// Estado interno
// =====================================================

static SafeCmdVel g_last_safe_cmd;

// =====================================================
// Utilidades
// =====================================================

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static float clamp_abs(float value, float abs_limit)
{
    return clamp_float(value, -abs_limit, abs_limit);
}

static bool valid_distance(float d)
{
    return d > 0.0f;
}

static void mark_limited(SafeCmdVel* cmd, uint8_t reason)
{
    if (cmd == nullptr) {
        return;
    }

    cmd->limited = true;

    if (cmd->reason == SAFETY_REASON_NONE) {
        cmd->reason = reason;
    }
}

// =====================================================
// Inicialización
// =====================================================

void safety_init()
{
    g_last_safe_cmd.v_m_s = 0.0f;
    g_last_safe_cmd.w_rad_s = 0.0f;

    g_last_safe_cmd.limited = false;
    g_last_safe_cmd.emergency_stop = false;
    g_last_safe_cmd.reason = SAFETY_REASON_NONE;

    g_last_safe_cmd.front_m = -1.0f;
    g_last_safe_cmd.left_m = -1.0f;
    g_last_safe_cmd.right_m = -1.0f;
}

// =====================================================
// Filtro de seguridad principal
// =====================================================

SafeCmdVel safety_filter_cmd_vel(
    float v_cmd_m_s,
    float w_cmd_rad_s,
    float front_m,
    float left_m,
    float right_m,
    bool front_ok,
    bool left_ok,
    bool right_ok
)
{
    SafeCmdVel out;

    out.v_m_s = v_cmd_m_s;
    out.w_rad_s = w_cmd_rad_s;

    out.limited = false;
    out.emergency_stop = false;
    out.reason = SAFETY_REASON_NONE;

    out.front_m = front_m;
    out.left_m = left_m;
    out.right_m = right_m;

    if (!SAFETY_ENABLE) {
        g_last_safe_cmd = out;
        return out;
    }

    // =================================================
    // 1. Límite absoluto de comandos
    // =================================================
    const float original_v = out.v_m_s;
    const float original_w = out.w_rad_s;

    out.v_m_s = clamp_abs(out.v_m_s, SAFETY_MAX_LINEAR_M_S);
    out.w_rad_s = clamp_abs(out.w_rad_s, SAFETY_MAX_ANGULAR_RAD_S);

    if ((fabs(out.v_m_s - original_v) > 0.0001f) ||
        (fabs(out.w_rad_s - original_w) > 0.0001f)) {
        mark_limited(&out, SAFETY_REASON_CMD_LIMITED);
    }

    // =================================================
    // 2. Seguridad frontal
    // =================================================
    //
    // Solo limita avance positivo.
    // El retroceso se permite para que el robot pueda salir
    // de una situación de bloqueo.
    //
    // =================================================
    const bool moving_forward = out.v_m_s > 0.0f;

    if (moving_forward) {

        const bool front_valid = front_ok && valid_distance(front_m);

        if (!front_valid) {
            if (SAFETY_FAILSAFE_ON_FRONT_INVALID) {
                out.v_m_s = 0.0f;
                out.emergency_stop = true;
                mark_limited(&out, SAFETY_REASON_FRONT_INVALID);
            }
        } else {

            // -----------------------------------------
            // Zona de parada
            // -----------------------------------------
            if (front_m <= SAFETY_FRONT_STOP_M) {
                out.v_m_s = 0.0f;
                out.emergency_stop = true;
                mark_limited(&out, SAFETY_REASON_FRONT_STOP);

                if (SAFETY_ALLOW_TURN_ON_FRONT_STOP) {
                    out.w_rad_s = clamp_abs(
                        out.w_rad_s,
                        SAFETY_FRONT_STOP_MAX_TURN_RAD_S
                    );
                } else {
                    out.w_rad_s = 0.0f;
                }
            }

            // -----------------------------------------
            // Zona de reducción progresiva
            // -----------------------------------------
            else if (front_m <= SAFETY_FRONT_SLOW_M) {
                const float span = SAFETY_FRONT_SLOW_M -
                                   SAFETY_FRONT_STOP_M;

                float factor = (front_m - SAFETY_FRONT_STOP_M) / span;

                factor = clamp_float(
                    factor,
                    SAFETY_MIN_SLOW_FACTOR,
                    1.0f
                );

                out.v_m_s = out.v_m_s * factor;
                mark_limited(&out, SAFETY_REASON_FRONT_SLOW);
            }
        }
    }

    // =================================================
    // 3. Seguridad lateral izquierda
    // =================================================
    //
    // Convención:
    // angular.z > 0 -> giro hacia izquierda.
    //
    // Si el sensor izquierdo detecta obstáculo cerca, se
    // limita o bloquea el giro hacia la izquierda.
    //
    // =================================================
    if (out.w_rad_s > 0.0f) {

        const bool left_valid = left_ok && valid_distance(left_m);

        if (!left_valid) {
            if (SAFETY_FAILSAFE_ON_SIDE_INVALID) {
                out.w_rad_s = 0.0f;
                mark_limited(&out, SAFETY_REASON_LEFT_INVALID);
            }
        } else {

            if (left_m <= SAFETY_SIDE_STOP_M) {
                out.w_rad_s = 0.0f;
                mark_limited(&out, SAFETY_REASON_LEFT_STOP);
            }

            else if (left_m <= SAFETY_SIDE_SLOW_M) {
                const float span = SAFETY_SIDE_SLOW_M -
                                   SAFETY_SIDE_STOP_M;

                float factor = (left_m - SAFETY_SIDE_STOP_M) / span;
                factor = clamp_float(factor, 0.20f, 1.0f);

                out.w_rad_s = out.w_rad_s * factor;
                mark_limited(&out, SAFETY_REASON_LEFT_STOP);
            }
        }
    }

    // =================================================
    // 4. Seguridad lateral derecha
    // =================================================
    //
    // Convención:
    // angular.z < 0 -> giro hacia derecha.
    //
    // =================================================
    if (out.w_rad_s < 0.0f) {

        const bool right_valid = right_ok && valid_distance(right_m);

        if (!right_valid) {
            if (SAFETY_FAILSAFE_ON_SIDE_INVALID) {
                out.w_rad_s = 0.0f;
                mark_limited(&out, SAFETY_REASON_RIGHT_INVALID);
            }
        } else {

            if (right_m <= SAFETY_SIDE_STOP_M) {
                out.w_rad_s = 0.0f;
                mark_limited(&out, SAFETY_REASON_RIGHT_STOP);
            }

            else if (right_m <= SAFETY_SIDE_SLOW_M) {
                const float span = SAFETY_SIDE_SLOW_M -
                                   SAFETY_SIDE_STOP_M;

                float factor = (right_m - SAFETY_SIDE_STOP_M) / span;
                factor = clamp_float(factor, 0.20f, 1.0f);

                out.w_rad_s = out.w_rad_s * factor;
                mark_limited(&out, SAFETY_REASON_RIGHT_STOP);
            }
        }
    }

    g_last_safe_cmd = out;
    return out;
}

// =====================================================
// Getters
// =====================================================

SafeCmdVel safety_get_last_cmd()
{
    return g_last_safe_cmd;
}

bool safety_is_limited()
{
    return g_last_safe_cmd.limited;
}

bool safety_is_emergency_stop()
{
    return g_last_safe_cmd.emergency_stop;
}

uint8_t safety_get_reason()
{
    return g_last_safe_cmd.reason;
}