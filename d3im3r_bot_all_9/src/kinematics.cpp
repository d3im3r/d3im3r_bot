#include "kinematics.h"
#include "app_config.h"

#include <math.h>

// =====================================================
// Cinemática inversa
// =====================================================
//
// Robot diferencial:
//
// w_left  = (v - w * L/2) / R
// w_right = (v + w * L/2) / R
//
// Donde:
// - v = velocidad lineal del robot [m/s]
// - w = velocidad angular del robot [rad/s]
// - L = distancia entre ruedas [m]
// - R = radio de rueda [m]
//
// =====================================================
WheelSpeedRefs kinematics_inverse(float v_m_s, float w_rad_s)
{
    WheelSpeedRefs refs;

    const float half_base = WHEEL_BASE_M * 0.5f;

    refs.left_rad_s  = (v_m_s - (half_base * w_rad_s)) / WHEEL_RADIUS_M;
    refs.right_rad_s = (v_m_s + (half_base * w_rad_s)) / WHEEL_RADIUS_M;

    kinematics_clamp_wheel_refs(&refs);

    return refs;
}

// =====================================================
// Alias para compatibilidad
// =====================================================
WheelSpeedRefs kinematics_vw_to_wheels(float v_m_s, float w_rad_s)
{
    return kinematics_inverse(v_m_s, w_rad_s);
}

// =====================================================
// Cinemática directa
// =====================================================
//
// Robot diferencial:
//
// v = R/2 * (w_right + w_left)
//
// w = R/L * (w_right - w_left)
//
// Donde:
// - w_left  = velocidad angular rueda izquierda [rad/s]
// - w_right = velocidad angular rueda derecha [rad/s]
// - v       = velocidad lineal del robot [m/s]
// - w       = velocidad angular del robot [rad/s]
//
// =====================================================
BodyTwist2D kinematics_forward(float left_rad_s, float right_rad_s)
{
    BodyTwist2D twist;

    twist.v_m_s = (WHEEL_RADIUS_M * 0.5f) * (right_rad_s + left_rad_s);

    twist.w_rad_s = (WHEEL_RADIUS_M / WHEEL_BASE_M) *
                    (right_rad_s - left_rad_s);

    return twist;
}

// =====================================================
// Límites de referencia de rueda
// =====================================================
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