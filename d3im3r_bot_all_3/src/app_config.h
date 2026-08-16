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
// Serial
// =========================
#define SERIAL_BAUDRATE       115200

// =========================
// Pines sensores
// =========================
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_FREQ_HZ           100000

#define XSHUT_CENTER_PIN       5
#define XSHUT_LEFT_PIN        14
#define XSHUT_RIGHT_PIN       23

// Direcciones nuevas VL53L0X
#define TOF_ADDR_CENTER      0x30
#define TOF_ADDR_RIGHT       0x31
#define TOF_ADDR_LEFT        0x32

// =========================
// Motores
// =========================
#define MOTOR_LEFT_PWM_PIN    13
#define MOTOR_LEFT_IN1_PIN    26
#define MOTOR_LEFT_IN2_PIN    27

#define MOTOR_RIGHT_PWM_PIN   25
#define MOTOR_RIGHT_IN1_PIN   33
#define MOTOR_RIGHT_IN2_PIN   32

// =========================
// PWM
// =========================
#define PWM_CH_LEFT           0
#define PWM_CH_RIGHT          1

#define PWM_FREQ_HZ           1000
#define PWM_RES_BITS          8

#define PWM_LIMIT_PERCENT       100.0f
#define PWM_DEADBAND_PERCENT    12.0f
#define MOTOR_TIMEOUT_MS        500

// Dominio PWM real usado por el controlador
#define CTRL_PWM_MAX          255.0f

// =========================
// Encoders
// =========================
#define ENCODER_LEFT_A_PIN     4
#define ENCODER_LEFT_B_PIN     2

#define ENCODER_RIGHT_A_PIN   18
#define ENCODER_RIGHT_B_PIN   19

#define ENCODER_LEFT_INVERTED   true
#define ENCODER_RIGHT_INVERTED  false

// =========================
// Parámetros físicos
// =========================
#define ENCODER_PPR          2112.0f
#define WHEEL_RADIUS_M       0.044f

// =====================================================
// Timing
// =====================================================
#define UROS_PUBLISH_PERIOD_MS        50//100
#define ENCODER_VEL_UPDATE_PERIOD_MS  20//100
#define CONTROL_PERIOD_MS             20//100
#define SENSORS_UPDATE_PERIOD_MS      50
#define OLED_UPDATE_PERIOD_MS         500

// =====================================================
// Control PI no incremental en dominio PWM
// u_pwm = Kp*e + Ki*integral(e)
// =====================================================
#define CTRL_LEFT_KP_PWM       9.6667f
#define CTRL_RIGHT_KP_PWM      9.6667f

// Empieza pequeño para no desestabilizar
#define CTRL_LEFT_KI_PWM       3.5f
#define CTRL_RIGHT_KI_PWM      3.5f

// Límite de la integral acumulada [rad]
#define CTRL_INTEGRAL_LIMIT    30.0f

#define CTRL_OUTPUT_LIMIT_PERCENT   100.0f

// Para detener limpio cuando la referencia sea cero
#define CTRL_REF_ZERO_EPSILON       0.05f

#endif