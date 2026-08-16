#ifndef SYNC_CONFIG_H
#define SYNC_CONFIG_H

// =====================================================
// Sincronía híbrida para avance recto
// =====================================================
//
// Esta capa corrige referencias de rueda cuando el robot
// debe avanzar recto.
//
// Se activa si:
//
// linear.x  != 0
// angular.z ≈ 0
//
// No debe intervenir en giros reales.
//
// =====================================================

#define SYNC_ENABLE                    true

// La sincronía solo se activa si el robot realmente avanza
// y el comando angular es prácticamente cero.
#define SYNC_MIN_LINEAR_CMD_M_S        0.03f
#define SYNC_ANGULAR_EPS_RAD_S         0.05f

// =====================================================
// Corrección basada en encoders
// =====================================================
//
// Si el robot se desvía a la izquierda, normalmente la rueda
// derecha está avanzando más que la izquierda.
//
// error_enc = w_left_meas - w_right_meas
// corr_enc  = Kp_enc * error_enc
//
// =====================================================

#define SYNC_ENC_ENABLE                true
#define SYNC_ENC_KP                    0.10f
#define SYNC_ENC_MAX_CORR_RAD_S        0.45f

// =====================================================
// Corrección basada en IMU
// =====================================================
//
// Cuando inicia un avance recto, se bloquea el yaw actual
// como referencia.
//
// yaw_error = yaw_ref - yaw_actual
//
// =====================================================

#define SYNC_IMU_ENABLE                true
#define SYNC_IMU_KP                    0.30f
#define SYNC_YAW_DEADBAND_RAD          0.020f
#define SYNC_IMU_MAX_CORR_RAD_S        0.45f

// =====================================================
// Límite total de corrección
// =====================================================

#define SYNC_TOTAL_MAX_CORR_RAD_S      0.60f

#endif