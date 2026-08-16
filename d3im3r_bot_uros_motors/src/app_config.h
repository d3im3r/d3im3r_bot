#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

// =====================================================
// WiFi / micro-ROS
// =====================================================
// static char WIFI_SSID[] = "Turtlebot";
// static char WIFI_PASS[] = "carlitosbot";

// static IPAddress AGENT_IP(192, 168, 1, 107);
static char WIFI_SSID[] = "Turtlebot";
static char WIFI_PASS[] = "carlitosbot";

static IPAddress AGENT_IP(192,168,1,102);
static const uint16_t AGENT_PORT = 8888;

// =====================================================
// Encoders
// Ajusta según tu robot
// =====================================================
#define ENCODER_LEFT_A_PIN     4
#define ENCODER_LEFT_B_PIN     2

#define ENCODER_RIGHT_A_PIN    18
#define ENCODER_RIGHT_B_PIN    19

#define ENCODER_LEFT_INVERTED   true
#define ENCODER_RIGHT_INVERTED  false

// =====================================================
// Publicación
// =====================================================
#define UROS_ENCODER_PUBLISH_PERIOD_MS  100

#endif