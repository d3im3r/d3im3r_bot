#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>

#include "app_config.h"
#include "motors.h"
#include "encoders.h"
#include "control.h"
#include "sensors.h"
#include "oled.h"
#include "uros.h"

static bool g_oled_ok = false;
static bool g_uros_ok = false;
static bool g_wifi_ok = false;

static uint32_t g_last_sensors_ms = 0;
static uint32_t g_last_oled_ms = 0;

static bool robot_is_stopped()
{
    return (fabs(control_get_left_ref()) < CTRL_REF_ZERO_EPSILON) &&
           (fabs(control_get_right_ref()) < CTRL_REF_ZERO_EPSILON);
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(1500);

    Serial.println();
    Serial.println("=== d3im3r BOT START ===");

    // =================================================
    // I2C
    // =================================================
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    delay(200);

    // =================================================
    // OLED
    // =================================================
    g_oled_ok = oled_init();

    if (!g_oled_ok) {
        Serial.println("[BOOT] ERROR OLED");
    }

    // =================================================
    // Módulos base
    // =================================================
    motors_init();
    encoders_init();
    control_init();

    // =================================================
    // Sensores
    // =================================================
    bool sensors_ok = sensors_init();
    (void)sensors_ok;

    // =================================================
    // WiFi / micro-ROS
    // =================================================
    g_uros_ok = uros_init();
    g_wifi_ok = (WiFi.status() == WL_CONNECTED);

    // =================================================
    // Pantalla de arranque final
    // =================================================
    if (g_oled_ok) {
        oled_show_boot_status(
            bno_ok,
            tof_center_ok,
            tof_left_ok,
            tof_right_ok,
            g_wifi_ok,
            g_uros_ok
        );

        delay(2000);
    }

    g_last_sensors_ms = millis();
    g_last_oled_ms = millis();

    Serial.println("[BOOT] SYSTEM READY");
}

void loop()
{
    // =================================================
    // 1. micro-ROS
    // =================================================
    if (g_uros_ok) {
        uros_spin();
    }

    // =================================================
    // 2. Encoders
    // =================================================
    encoders_update();

    const uint32_t now = millis();

    // =================================================
    // 3. Sensores
    // =================================================
    //
    // La IMU se actualiza antes de recalcular la corrección
    // de línea recta. Así, la capa sync trabaja con el yaw
    // más reciente disponible.
    //
    // =================================================
    if ((now - g_last_sensors_ms) >= SENSORS_UPDATE_PERIOD_MS) {
        g_last_sensors_ms = now;

        sensors_update_imu();
        sensors_update_tof();
    }

    // =================================================
    // 4. Actualización continua del comando de movimiento
    // =================================================
    //
    // Aquí se recalculan continuamente:
    //
    // /cmd_vel
    //    ↓
    // cinemática diferencial
    //    ↓
    // corrección encoder + IMU para línea recta
    //    ↓
    // referencias finales al PI
    //
    // Debe ejecutarse después de encoders_update() y sensores,
    // pero antes de control_update().
    //
    // =================================================
    if (g_uros_ok) {
        uros_update_motion_command();
    }

    // =================================================
    // 5. Control PI incremental y motores
    // =================================================
    control_update();
    motors_update();

    // =================================================
    // 6. OLED
    // =================================================
    if (g_oled_ok && (now - g_last_oled_ms) >= OLED_UPDATE_PERIOD_MS) {
        g_last_oled_ms = now;

        float yaw_deg = imu_yaw * 180.0f / PI;

        oled_show_runtime(
            tof_distances[0],
            tof_distances[1],
            tof_distances[2],
            yaw_deg,
            bno_ok,
            encoders_get_left_rad_s(),
            encoders_get_right_rad_s(),
            calibration_status,
            robot_is_stopped()
        );
    }

    delay(2);
}