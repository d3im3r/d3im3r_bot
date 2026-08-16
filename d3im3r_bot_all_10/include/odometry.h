#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "kinematics.h"

// =====================================================
// Pose 2D del robot
// =====================================================
typedef struct
{
    float x_m;
    float y_m;
    float theta_rad;
} OdomPose2D;

// =====================================================
// Estado completo de odometría ligera
// =====================================================
typedef struct
{
    OdomPose2D pose;
    BodyTwist2D twist;
} OdomState2D;

// Inicialización de odometría
void odometry_init();

// Reinicia x, y, theta y referencias internas.
// Útil si se quiere poner la odometría en cero.
void odometry_reset();

// Actualiza odometría a partir de velocidades de rueda medidas.
// Si imu_available es true y ODOM_USE_IMU_YAW está activo,
// theta se toma desde la IMU.
void odometry_update(
    float left_meas_rad_s,
    float right_meas_rad_s,
    float imu_yaw_rad,
    bool imu_available
);

// Lectura del estado estimado
OdomState2D odometry_get_state();
OdomPose2D odometry_get_pose();
BodyTwist2D odometry_get_twist();

// Lecturas escalares
float odometry_get_x_m();
float odometry_get_y_m();
float odometry_get_theta_rad();
float odometry_get_v_m_s();
float odometry_get_w_rad_s();

#endif