#include <Arduino.h>
#include "encoders.h"
#include "uros.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Iniciando d3im3r - encoders");

    if (!encoders_init()) {
        Serial.println("Error inicializando encoders");
    } else {
        Serial.println("Encoders OK");
    }

    if (!uros_init()) {
        Serial.println("Error inicializando micro-ROS");
    } else {
        Serial.println("micro-ROS OK");
    }
}

void loop()
{
    encoders_update();
    uros_spin();

    static uint32_t last_print = 0;
    if (millis() - last_print > 500) {
        last_print = millis();

        Serial.print("L ticks: ");
        Serial.print(encoders_get_left_ticks());
        Serial.print(" | R ticks: ");
        Serial.print(encoders_get_right_ticks());

        Serial.print(" | L rad/s: ");
        Serial.print(encoders_get_left_rad_s(), 4);
        Serial.print(" | R rad/s: ");
        Serial.print(encoders_get_right_rad_s(), 4);

        Serial.print(" | L rpm: ");
        Serial.print(encoders_get_left_rpm(), 2);
        Serial.print(" | R rpm: ");
        Serial.print(encoders_get_right_rpm(), 2);

        Serial.print(" | L m/s: ");
        Serial.print(encoders_get_left_m_s(), 4);
        Serial.print(" | R m/s: ");
        Serial.println(encoders_get_right_m_s(), 4);
    }

    delay(10);
}