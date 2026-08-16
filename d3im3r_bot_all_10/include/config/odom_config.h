#ifndef ODOM_CONFIG_H
#define ODOM_CONFIG_H

// =====================================================
// Configuración de odometría ligera
// =====================================================
//
// Esta capa estima:
// - x [m]
// - y [m]
// - yaw/theta [rad]
// - velocidad lineal real [m/s]
// - velocidad angular real [rad/s]
//
// La odometría NO controla el robot.
// Solo estima y publica su estado.
//
// =====================================================

// Usar IMU para la orientación de la odometría.
// true  -> theta se toma desde imu_yaw, compensado con offset inicial.
// false -> theta se integra solo con encoders.
#define ODOM_USE_IMU_YAW              true

// Si está activo, el yaw inicial se toma como cero.
// Esto hace que /odom_pose.z empiece cerca de 0 rad al arrancar.
#define ODOM_ZERO_YAW_ON_INIT         true

// Tiempo máximo permitido entre actualizaciones.
// Si por alguna razón el dt sale muy grande, se limita para evitar
// saltos bruscos en x, y, theta.
#define ODOM_MAX_DT_S                 0.20f

// Si el periodo es demasiado pequeño, se ignora la actualización.
#define ODOM_MIN_DT_S                 0.001f

// Si quieres depurar por Serial la pose estimada,
// puedes activar esto manualmente.
#define ODOM_DEBUG_SERIAL             false

#endif