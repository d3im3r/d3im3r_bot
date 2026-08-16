#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

// =====================================================
// WiFi / micro-ROS
// =====================================================
static char WIFI_SSID[] = "Turtlebot";
static char WIFI_PASS[] = "carlitosbot";

static IPAddress AGENT_IP(192, 168, 1, 102);
static const uint16_t AGENT_PORT = 8888;

// =====================================================
// Pines I2C y sensores
// =====================================================
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

#define XSHUT_CENTER_PIN     5
#define XSHUT_LEFT_PIN      14
#define XSHUT_RIGHT_PIN     23

// =====================================================
// Motores
// =====================================================
#define MOTOR_LEFT_PWM_PIN   25
#define MOTOR_LEFT_IN1_PIN   33
#define MOTOR_LEFT_IN2_PIN   32

#define MOTOR_RIGHT_PWM_PIN  13
#define MOTOR_RIGHT_IN1_PIN  26
#define MOTOR_RIGHT_IN2_PIN  27

// =====================================================
// Encoders
// =====================================================
#define ENCODER_LEFT_A_PIN    4
#define ENCODER_LEFT_B_PIN    2

#define ENCODER_RIGHT_A_PIN  18
#define ENCODER_RIGHT_B_PIN  19

// Cambia a true si el sentido queda invertido
#define ENCODER_LEFT_INVERTED   true
#define ENCODER_RIGHT_INVERTED  false

// =====================================================
// Parámetros físicos
// =====================================================
#define ENCODER_PPR            2112.0f
#define WHEEL_RADIUS_M         0.044f

// =====================================================
// Publicación / cálculo
// =====================================================
#define UROS_ENCODER_PUBLISH_PERIOD_MS   50
#define ENCODER_VEL_UPDATE_PERIOD_MS     50

#endif