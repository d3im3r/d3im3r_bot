#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// =====================================================
// Pines I2C
// =====================================================
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_FREQ_HZ           100000

// =====================================================
// Pines XSHUT sensores VL53L0X
// =====================================================
#define XSHUT_CENTER_PIN       5
#define XSHUT_LEFT_PIN        14
#define XSHUT_RIGHT_PIN       23

// =====================================================
// Direcciones I2C sensores VL53L0X
// =====================================================
#define TOF_ADDR_CENTER      0x30
#define TOF_ADDR_RIGHT       0x31
#define TOF_ADDR_LEFT        0x32

// =====================================================
// Pines motores
// =====================================================
#define MOTOR_LEFT_PWM_PIN    13
#define MOTOR_LEFT_IN1_PIN    26
#define MOTOR_LEFT_IN2_PIN    27

#define MOTOR_RIGHT_PWM_PIN   25
#define MOTOR_RIGHT_IN1_PIN   33
#define MOTOR_RIGHT_IN2_PIN   32

// =====================================================
// Pines encoders
// =====================================================
#define ENCODER_LEFT_A_PIN     4
#define ENCODER_LEFT_B_PIN     2

#define ENCODER_RIGHT_A_PIN   18
#define ENCODER_RIGHT_B_PIN   19

#define ENCODER_LEFT_INVERTED   true
#define ENCODER_RIGHT_INVERTED  false

#endif
