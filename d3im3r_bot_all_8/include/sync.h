#ifndef SYNC_H
#define SYNC_H

#include <Arduino.h>
#include "kinematics.h"

// =====================================================
// Salida de la capa de sincronía
// =====================================================
typedef struct
{
    // Referencias finales corregidas para cada rueda
    float left_rad_s;
    float right_rad_s;

    // Indica si la corrección estuvo activa
    bool active;

    // Información de corrección por encoders
    float encoder_error_rad_s;
    float encoder_corr_rad_s;

    // Información de corrección por IMU
    float yaw_error_rad;
    float imu_corr_rad_s;

    // Corrección total aplicada
    float total_corr_rad_s;

} SyncOutput;

// Determina si la sincronía debe activarse según el comando
bool sync_should_activate(float v_cmd_m_s, float w_cmd_rad_s);

// Aplica corrección híbrida para avance recto.
// Esta función debe llamarse de forma continua mientras el
// robot esté ejecutando comandos por /cmd_vel.
SyncOutput sync_apply_hybrid_straight_correction(
    float left_ref_rad_s,
    float right_ref_rad_s,
    float left_meas_rad_s,
    float right_meas_rad_s,
    float v_cmd_m_s,
    float w_cmd_rad_s,
    float yaw_now_rad,
    bool imu_available
);

// Reinicia el bloqueo de rumbo
void sync_reset();

#endif