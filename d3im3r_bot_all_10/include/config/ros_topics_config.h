#ifndef ROS_TOPICS_CONFIG_H
#define ROS_TOPICS_CONFIG_H

// =====================================================
// Identidad del nodo micro-ROS
// =====================================================
#define UROS_NODE_NAME "d3im3r_robot_node"
#define UROS_NODE_NAMESPACE ""

// =====================================================
// Tópicos publicados por el ESP32
// =====================================================
//
// geometry_msgs/msg/Vector3
// x = velocidad rueda izquierda [rad/s]
// y = velocidad rueda derecha [rad/s]
// z = reservado
#define TOPIC_WHEEL_VELOCITIES_RAD_S "wheel_velocities_rad_s"

// std_msgs/msg/Float32
// data = acción de control de la rueda izquierda [%]
#define TOPIC_LEFT_CONTROL_ACTION "left_control_action"

// std_msgs/msg/Float32
// data = acción de control de la rueda derecha [%]
#define TOPIC_RIGHT_CONTROL_ACTION "right_control_action"

// std_msgs/msg/Float32
// data = yaw medido por IMU [rad]
#define TOPIC_IMU_YAW_RAD "imu_yaw_rad"

// geometry_msgs/msg/Vector3
// x = distancia frontal [m]
// y = distancia izquierda [m]
// z = distancia derecha [m]
// Valor -1.0 indica sensor no disponible o lectura inválida.
#define TOPIC_TOF_DISTANCES_M "tof_distances_m"

// geometry_msgs/msg/Vector3
// x = posición x estimada [m]
// y = posición y estimada [m]
// z = orientación yaw/theta estimada [rad]
#define TOPIC_ODOM_POSE "odom_pose"

// geometry_msgs/msg/Vector3
// x = velocidad lineal estimada del robot [m/s]
// y = velocidad angular estimada del robot [rad/s]
// z = reservado
#define TOPIC_ODOM_TWIST "odom_twist"

// geometry_msgs/msg/Vector3
// x = limited
//     0.0 = comando sin limitar
//     1.0 = comando limitado por seguridad
//
// y = emergency_stop
//     0.0 = sin parada de emergencia
//     1.0 = parada de emergencia activa
//
// z = reason
//     0 = none
//     1 = front slow
//     2 = front stop
//     3 = front invalid
//     4 = left stop
//     5 = right stop
//     6 = left invalid
//     7 = right invalid
//     8 = command limited
#define TOPIC_SAFETY_STATUS "safety_status"

// =====================================================
// Tópicos suscritos por el ESP32
// =====================================================
//
// geometry_msgs/msg/Vector3
// x = referencia rueda izquierda [rad/s]
// y = referencia rueda derecha [rad/s]
// z = reservado
#define TOPIC_WHEEL_REFS_RAD_S "wheel_refs_rad_s"

// geometry_msgs/msg/Twist
// linear.x  = velocidad lineal del robot [m/s]
// angular.z = velocidad angular del robot [rad/s]
#define TOPIC_CMD_VEL "cmd_vel"

#endif