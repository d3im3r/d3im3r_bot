#pragma once

#include <Arduino.h>

enum OledBootStage : uint8_t
{
    OLED_BOOT_START = 0,
    OLED_BOOT_I2C,
    OLED_BOOT_BNO,
    OLED_BOOT_TOF_RIGHT,
    OLED_BOOT_TOF_LEFT,
    OLED_BOOT_TOF_CENTER,
    OLED_BOOT_UROS,
    OLED_BOOT_READY,
    OLED_BOOT_ERROR
};

bool oled_init();

void oled_show_boot(
    OledBootStage stage,
    bool bno_ok,
    bool tof_right_ok,
    bool tof_left_ok,
    bool tof_center_ok,
    bool uros_ok,
    const char* msg
);

void oled_show_runtime(
    float d_center_m,
    float d_left_m,
    float d_right_m,
    bool center_ok,
    bool left_ok,
    bool right_ok,
    float yaw_deg,
    bool yaw_ok,
    uint8_t cal_sys,
    uint8_t cal_gyro,
    uint8_t cal_accel,
    uint8_t cal_mag
);