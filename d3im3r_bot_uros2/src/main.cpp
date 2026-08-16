#include <Arduino.h>
#include <Wire.h>

#include "sensors.h"
#include "uros.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("[BOOT] Inicio");

    if(!sensors_init())
    {
        Serial.println("[BOOT] ERROR sensores");
        while(1);
    }

    Serial.println("[BOOT] Sensores OK");

    if(!uros_init())
    {
        Serial.println("[BOOT] ERROR microROS");
    }
}

void loop()
{
    sensors_update();

    uros_publish_imu(imu_qx, imu_qy, imu_qz, imu_qw);
    uros_publish_calibration(calibration_status);

    uros_publish_tof(
        tof_distances[0],
        tof_distances[1],
        tof_distances[2]
    );

    uros_spin();

    delay(50);
}