#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "uros.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Iniciando d3im3r base");

    if (!encoders_init()) {
        Serial.println("Error inicializando encoders");
    } else {
        Serial.println("Encoders OK");
    }

    if (!motors_init()) {
        Serial.println("Error inicializando motores");
    } else {
        Serial.println("Motores OK");
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
    motors_update();
    uros_spin();

    static uint32_t last_print = 0;
    if ((millis() - last_print) > 500) {
        last_print = millis();

        Serial.print("L ticks: ");
        Serial.print(encoders_get_left_ticks());
        Serial.print(" | R ticks: ");
        Serial.print(encoders_get_right_ticks());

        Serial.print(" | L rpm: ");
        Serial.print(encoders_get_left_rpm(), 2);
        Serial.print(" | R rpm: ");
        Serial.print(encoders_get_right_rpm(), 2);

        Serial.print(" | PWM L: ");
        Serial.print(motors_get_left_applied(), 1);
        Serial.print(" | PWM R: ");
        Serial.println(motors_get_right_applied(), 1);
    }

    delay(10);
}