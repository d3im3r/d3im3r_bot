#include "safety.h"
#include "app_config.h"

#include <math.h>

static SafeCmdVel g_last_safe_cmd;

static float clamp_float(float value, float min_val, float max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

static float clamp_abs(float value, float abs_limit)
{
    return clamp_float(value, -abs_limit, abs_limit);
}

static inline bool valid_distance(float d)
{
    return d > 0.0f;
}

static void mark_limited(SafeCmdVel &cmd, uint8_t reason)
{
    cmd.limited = true;
    if (cmd.reason == SAFETY_REASON_NONE) {
        cmd.reason = reason;
    }
}

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

static void check_side_safety(
    float &w_rad_s,
    float dist_m,
    bool sensor_ok,
    uint8_t stop_reason,
    uint8_t invalid_reason,
    SafeCmdVel &out
)
{
    const bool valid = sensor_ok && valid_distance(dist_m);

    if (!valid) {
        if (SAFETY_FAILSAFE_ON_SIDE_INVALID) {
            w_rad_s = 0.0f;
            mark_limited(out, invalid_reason);
        }
    } else {
        if (dist_m <= SAFETY_SIDE_STOP_M) {
            w_rad_s = 0.0f;
            mark_limited(out, stop_reason);
        } else if (dist_m <= SAFETY_SIDE_SLOW_M) {
            const float span = SAFETY_SIDE_SLOW_M - SAFETY_SIDE_STOP_M;
            float factor = (dist_m - SAFETY_SIDE_STOP_M) / span;
            factor = clamp_float(factor, 0.20f, 1.0f);
            w_rad_s *= factor;
            mark_limited(out, SAFETY_REASON_CMD_LIMITED);
        }
    }
}

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

    // 1. Límite absoluto
    const float orig_v = out.v_m_s;
    const float orig_w = out.w_rad_s;
    out.v_m_s = clamp_abs(out.v_m_s, SAFETY_MAX_LINEAR_M_S);
    out.w_rad_s = clamp_abs(out.w_rad_s, SAFETY_MAX_ANGULAR_RAD_S);

    if ((fabs(out.v_m_s - orig_v) > 0.0001f) || (fabs(out.w_rad_s - orig_w) > 0.0001f)) {
        mark_limited(out, SAFETY_REASON_CMD_LIMITED);
    }

    // 2. Seguridad frontal
    if (out.v_m_s > 0.0f) {
        const bool front_valid = front_ok && valid_distance(front_m);

        if (!front_valid) {
            if (SAFETY_FAILSAFE_ON_FRONT_INVALID) {
                out.v_m_s = 0.0f;
                out.emergency_stop = true;
                mark_limited(out, SAFETY_REASON_FRONT_INVALID);
            }
        } else {
            if (front_m <= SAFETY_FRONT_STOP_M) {
                out.v_m_s = 0.0f;
                out.emergency_stop = true;
                mark_limited(out, SAFETY_REASON_FRONT_STOP);

                if (SAFETY_ALLOW_TURN_ON_FRONT_STOP) {
                    out.w_rad_s = clamp_abs(out.w_rad_s, SAFETY_FRONT_STOP_MAX_TURN_RAD_S);
                } else {
                    out.w_rad_s = 0.0f;
                }
            } else if (front_m <= SAFETY_FRONT_SLOW_M) {
                const float span = SAFETY_FRONT_SLOW_M - SAFETY_FRONT_STOP_M;
                float factor = (front_m - SAFETY_FRONT_STOP_M) / span;
                factor = clamp_float(factor, SAFETY_MIN_SLOW_FACTOR, 1.0f);
                out.v_m_s *= factor;
                mark_limited(out, SAFETY_REASON_FRONT_SLOW);
            }
        }
    }

    // 3. Seguridad lateral izquierda (w > 0)
    if (out.w_rad_s > 0.0f) {
        check_side_safety(
            out.w_rad_s, left_m, left_ok,
            SAFETY_REASON_LEFT_STOP, SAFETY_REASON_LEFT_INVALID, out
        );
    }

    // 4. Seguridad lateral derecha (w < 0)
    if (out.w_rad_s < 0.0f) {
        check_side_safety(
            out.w_rad_s, right_m, right_ok,
            SAFETY_REASON_RIGHT_STOP, SAFETY_REASON_RIGHT_INVALID, out
        );
    }

    g_last_safe_cmd = out;
    return out;
}

SafeCmdVel safety_get_last_cmd()   { return g_last_safe_cmd; }
bool safety_is_limited()           { return g_last_safe_cmd.limited; }
bool safety_is_emergency_stop()   { return g_last_safe_cmd.emergency_stop; }
uint8_t safety_get_reason()        { return g_last_safe_cmd.reason; }