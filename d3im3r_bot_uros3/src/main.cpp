#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "sensors.h"
#include "uros.h"
#include "oled.h"

const uint32_t BNO_PERIOD  = 50;   // 20 Hz
const uint32_t TOF_PERIOD  = 66;   // 15 Hz
const uint32_t ROS_PERIOD  = 100;  // 10 Hz
const uint32_t OLED_PERIOD = 250;  // 4 Hz

uint32_t t_bno  = 0;
uint32_t t_tof  = 0;
uint32_t t_ros  = 0;
uint32_t t_oled = 0;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Wire.begin(21, 22);
    Wire.setClock(100000);

    oled_init();
    oled_show_boot(
        OLED_BOOT_START,
        false, false, false, false, false,
        "Iniciando..."
    );

    oled_show_boot(
        OLED_BOOT_I2C,
        false, false, false, false, false,
        "I2C OK"
    );
    delay(250);

    oled_show_boot(
        OLED_BOOT_BNO,
        false, false, false, false, false,
        "Chequeando BNO"
    );

    if (!sensors_init())
    {
        oled_show_boot(
            OLED_BOOT_ERROR,
            bno_ok, tof_right_ok, tof_left_ok, tof_center_ok, false,
            "Sensor init FAIL"
        );

        Serial.println("Sensor init failed");
        while (1) { delay(100); }
    }

    oled_show_boot(
        OLED_BOOT_TOF_CENTER,
        bno_ok, tof_right_ok, tof_left_ok, tof_center_ok, false,
        "Sensores OK"
    );
    delay(400);

    oled_show_boot(
        OLED_BOOT_UROS,
        bno_ok, tof_right_ok, tof_left_ok, tof_center_ok, false,
        "Init microROS"
    );

    bool uros_ok = uros_init();

    if (!uros_ok)
    {
        oled_show_boot(
            OLED_BOOT_ERROR,
            bno_ok, tof_right_ok, tof_left_ok, tof_center_ok, false,
            "microROS FAIL"
        );

        Serial.println("microROS init failed");
        delay(800);
    }
    else
    {
        oled_show_boot(
            OLED_BOOT_READY,
            bno_ok, tof_right_ok, tof_left_ok, tof_center_ok, true,
            "Sistema listo"
        );
        delay(600);
    }
}

void loop()
{
    uint32_t now = millis();

    // IMU 20 Hz
    if (now - t_bno >= BNO_PERIOD)
    {
        sensors_update_imu();
        t_bno = now;
    }

    // TOF 15 Hz
    if (now - t_tof >= TOF_PERIOD)
    {
        sensors_update_tof();
        t_tof = now;
    }

    // ROS 10 Hz
    if (now - t_ros >= ROS_PERIOD)
    {
        uros_publish_orientation(imu_qx, imu_qy, imu_qz, imu_qw);
        uros_publish_yaw(imu_yaw);
        uros_publish_calibration(calibration_status);
        uros_publish_tof(
            tof_distances[0],
            tof_distances[1],
            tof_distances[2]
        );

        t_ros = now;
    }

    // OLED 4 Hz
    if (now - t_oled >= OLED_PERIOD)
    {
        float yaw_deg = imu_yaw * 180.0f / PI;
        bool yaw_ok = !isnan(yaw_deg);

        oled_show_runtime(
            tof_distances[0],
            tof_distances[1],
            tof_distances[2],
            tof_center_ok,
            tof_left_ok,
            tof_right_ok,
            yaw_deg,
            yaw_ok,
            cal_sys,
            cal_gyro,
            cal_accel,
            cal_mag
        );

        t_oled = now;
    }

    uros_spin();
}