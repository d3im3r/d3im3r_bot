#include <Arduino.h>
#include "encoders.h"
#include "uros.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Iniciando d3im3r - modulo encoders");

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
    uros_spin();

    static uint32_t last_print = 0;
    if (millis() - last_print > 500) {
        last_print = millis();

        Serial.print("L: ");
        Serial.print(encoders_get_left_ticks());
        Serial.print(" | R: ");
        Serial.println(encoders_get_right_ticks());
    }

    delay(10);
}