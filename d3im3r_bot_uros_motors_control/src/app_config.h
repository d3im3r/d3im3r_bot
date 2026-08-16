#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

// =========================
// WiFi / micro-ROS
// =========================
static char WIFI_SSID[] = "Turtlebot";
static char WIFI_PASS[] = "carlitosbot";

static IPAddress AGENT_IP(192, 168, 1, 102);
static const uint16_t AGENT_PORT = 8888;

// =========================
// Pines sensores
// =========================
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

#define XSHUT_CENTER_PIN     5
#define XSHUT_LEFT_PIN      14
#define XSHUT_RIGHT_PIN     23

// =========================
// Motores
// =========================
#define MOTOR_LEFT_PWM_PIN   13
#define MOTOR_LEFT_IN1_PIN   26
#define MOTOR_LEFT_IN2_PIN   27

#define MOTOR_RIGHT_PWM_PIN  25
#define MOTOR_RIGHT_IN1_PIN  33
#define MOTOR_RIGHT_IN2_PIN  32

// =========================
// PWM
// =========================
#define PWM_CH_LEFT          0
#define PWM_CH_RIGHT         1

#define PWM_FREQ_HZ          1000
#define PWM_RES_BITS         8

#define PWM_LIMIT_PERCENT      100.0f
#define PWM_DEADBAND_PERCENT   12.0f
#define MOTOR_TIMEOUT_MS       500

// =========================
// Encoders
// =========================
#define ENCODER_LEFT_A_PIN    4
#define ENCODER_LEFT_B_PIN    2

#define ENCODER_RIGHT_A_PIN  18
#define ENCODER_RIGHT_B_PIN  19

#define ENCODER_LEFT_INVERTED    true
#define ENCODER_RIGHT_INVERTED   false

// =========================
// Parámetros físicos
// =========================
#define ENCODER_PPR         2112.0f
#define WHEEL_RADIUS_M      0.044f

// =====================================================
// Timing
// =====================================================
#define UROS_PUBLISH_PERIOD_MS       100
#define ENCODER_VEL_UPDATE_PERIOD_MS 100
#define CONTROL_PERIOD_MS            100

// =====================================================
// Control PI incremental
// u[k] = u[k-1] + q0*e[k] + q1*e[k-1]
// =====================================================
#define USE_INCREMENTAL_PI   1

// Si vuelves a P puro, estos quedan disponibles
#define CTRL_LEFT_KP         9.6667f
#define CTRL_RIGHT_KP        9.6667f

// Ganancias iniciales PI
#define CTRL_LEFT_Q0         9.6667f
#define CTRL_LEFT_Q1        -7.83333f

#define CTRL_RIGHT_Q0        9.6667f
#define CTRL_RIGHT_Q1       -7.83333f

#define CTRL_OUTPUT_LIMIT_PERCENT  100.0f

#endif