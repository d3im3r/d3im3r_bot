#ifndef SYNC_CONFIG_H
#define SYNC_CONFIG_H

// =====================================================
// Sincronía híbrida para avance recto
// =====================================================
//
// Esta capa NO reemplaza al PI de velocidad de cada rueda.
// Su función es corregir las referencias de velocidad antes
// de enviarlas al PI, con el objetivo de mantener trayectoria
// recta cuando el robot recibe un comando:
//
// linear.x  != 0
// angular.z ≈ 0
//
// Flujo:
//
// /cmd_vel
//    ↓
// cinemática diferencial
//    ↓
// referencias base de rueda
//    ↓
// corrección encoder + IMU
//    ↓
// PI incremental de cada rueda
//    ↓
// motores
//
// =====================================================

#define SYNC_ENABLE                    true

// La sincronía solo se activa si el comando lineal supera
// este valor y el comando angular es prácticamente cero.
#define SYNC_MIN_LINEAR_CMD_M_S        0.03f
#define SYNC_ANGULAR_EPS_RAD_S         0.10f

// =====================================================
// Corrección basada en encoders
// =====================================================
//
// Compara las velocidades medidas de ambas ruedas.
// Si una rueda se adelanta respecto a la otra, modifica
// ligeramente las referencias.
//
// error_enc = w_left_meas - w_right_meas
//
// corr_enc = Kp_enc * error_enc
//
// =====================================================

#define SYNC_ENC_ENABLE                true
#define SYNC_ENC_KP                    0.20f
#define SYNC_ENC_MAX_CORR_RAD_S        0.80f

// =====================================================
// Corrección basada en IMU
// =====================================================
//
// Cuando inicia un avance recto, se bloquea el yaw actual
// como referencia:
//
// yaw_ref = yaw_actual
//
// Luego se calcula:
//
// yaw_error = yaw_ref - yaw_actual
//
// Si el robot se desvía, se corrigen las referencias de
// las ruedas para recuperar el rumbo.
//
// =====================================================

#define SYNC_IMU_ENABLE                true

// Ganancia inicial conservadora.
// Si la corrección es muy débil, probar 0.80f o 1.00f.
// Si oscila, bajar a 0.40f.
#define SYNC_IMU_KP                    0.70f

// Zona muerta de yaw.
// 0.015 rad ≈ 0.86 grados.
#define SYNC_YAW_DEADBAND_RAD          0.015f

// Corrección máxima por IMU en rad/s.
#define SYNC_IMU_MAX_CORR_RAD_S        0.70f

// =====================================================
// Límite total de corrección
// =====================================================
//
// La corrección total es:
//
// total_corr = corr_encoder + corr_imu
//
// Este límite evita que la capa de sincronía modifique
// demasiado las referencias generadas por la cinemática.
//
// =====================================================

#define SYNC_TOTAL_MAX_CORR_RAD_S      1.20f

#endif