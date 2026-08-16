#ifndef UROS_H
#define UROS_H

#include <Arduino.h>

// =====================================================
// Interfaz micro-ROS del d3im3r_bot
// =====================================================
//
// Publica:
// - /wheel_velocities_rad_s : geometry_msgs/msg/Vector3
// - /left_control_action    : std_msgs/msg/Float32
// - /right_control_action   : std_msgs/msg/Float32
// - /imu_yaw_rad            : std_msgs/msg/Float32
// - /tof_distances_m        : geometry_msgs/msg/Vector3
// - /odom_pose              : geometry_msgs/msg/Vector3
// - /odom_twist             : geometry_msgs/msg/Vector3
// - /safety_status          : geometry_msgs/msg/Vector3
//
// Suscribe:
// - /wheel_refs_rad_s       : geometry_msgs/msg/Vector3
// - /cmd_vel                : geometry_msgs/msg/Twist
//
// Nota:
// Los nombres se configuran en include/config/ros_topics_config.h
//
// Modo de operación:
// - Si llega /wheel_refs_rad_s, se aplican referencias directas.
// - Si llega /cmd_vel, primero se filtra por seguridad ToF.
// - Luego se activa cinemática inversa.
// - Después se aplica corrección de línea recta encoder + IMU.
//
// =====================================================

bool uros_init();
void uros_spin();

// Debe llamarse dentro del loop(), después de actualizar
// encoders y sensores, y antes de control_update().
void uros_update_motion_command();

#endif