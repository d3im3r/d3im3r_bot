#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

// =====================================================
// Referencias de velocidad de rueda
// =====================================================
typedef struct
{
    float left_rad_s;
    float right_rad_s;
} WheelSpeedRefs;

// =====================================================
// Velocidad del cuerpo del robot
// =====================================================
//
// v_m_s:
// Velocidad lineal del robot [m/s]
//
// w_rad_s:
// Velocidad angular del robot [rad/s]
//
// =====================================================
typedef struct
{
    float v_m_s;
    float w_rad_s;
} BodyTwist2D;

// =====================================================
// Cinemática inversa
// =====================================================
//
// Convierte:
//
// v, w  ->  w_left, w_right
//
// Se usa para transformar /cmd_vel en referencias de rueda.
//
// =====================================================
WheelSpeedRefs kinematics_inverse(float v_m_s, float w_rad_s);

// Alias conservado para compatibilidad con el código anterior.
WheelSpeedRefs kinematics_vw_to_wheels(float v_m_s, float w_rad_s);

// =====================================================
// Cinemática directa
// =====================================================
//
// Convierte:
//
// w_left, w_right  ->  v, w
//
// Se usa para estimar la velocidad real del robot a partir
// de las velocidades medidas por los encoders.
//
// =====================================================
BodyTwist2D kinematics_forward(float left_rad_s, float right_rad_s);

// =====================================================
// Utilidades de límites
// =====================================================
float kinematics_clamp_wheel_ref(float wheel_rad_s);
void kinematics_clamp_wheel_refs(WheelSpeedRefs* refs);

#endif