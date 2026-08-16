#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

// =====================================================
// Códigos de estado de seguridad
// =====================================================

#define SAFETY_REASON_NONE                 0
#define SAFETY_REASON_FRONT_SLOW           1
#define SAFETY_REASON_FRONT_STOP           2
#define SAFETY_REASON_FRONT_INVALID        3
#define SAFETY_REASON_LEFT_STOP            4
#define SAFETY_REASON_RIGHT_STOP           5
#define SAFETY_REASON_LEFT_INVALID         6
#define SAFETY_REASON_RIGHT_INVALID        7
#define SAFETY_REASON_CMD_LIMITED          8

// =====================================================
// Estructura de comando seguro
// =====================================================

typedef struct
{
    float v_m_s;
    float w_rad_s;

    bool limited;
    bool emergency_stop;

    uint8_t reason;

    float front_m;
    float left_m;
    float right_m;

} SafeCmdVel;

// Inicialización de seguridad
void safety_init();

// Filtra un comando tipo /cmd_vel usando sensores ToF.
SafeCmdVel safety_filter_cmd_vel(
    float v_cmd_m_s,
    float w_cmd_rad_s,
    float front_m,
    float left_m,
    float right_m,
    bool front_ok,
    bool left_ok,
    bool right_ok
);

// Último estado calculado
SafeCmdVel safety_get_last_cmd();

// Utilidades de lectura rápida
bool safety_is_limited();
bool safety_is_emergency_stop();
uint8_t safety_get_reason();

#endif