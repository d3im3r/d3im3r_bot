#pragma once

#include <Arduino.h>

bool oled_init();

void oled_show_boot_status(
    bool bno_ok,
    bool center_ok,
    bool left_ok,
    bool right_ok,
    bool wifi_ok,
    bool uros_ok
);

void oled_show_runtime(
    float d_center_m,
    float d_left_m,
    float d_right_m,
    float yaw_deg,
    bool yaw_ok,
    float vel_left_rad_s,
    float vel_right_rad_s,
    uint16_t calibration_status,
    bool robot_stopped
);