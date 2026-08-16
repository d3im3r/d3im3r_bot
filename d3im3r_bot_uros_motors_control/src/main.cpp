#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "uros.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Iniciando d3im3r velocity control");

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

    if (!control_init()) {
        Serial.println("Error inicializando control");
    } else {
        Serial.println("Control OK");
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
    control_update();
    motors_update();
    uros_spin();

    static uint32_t last_print = 0;
    if ((millis() - last_print) > 500) {
        last_print = millis();

        Serial.print("L ref: ");
        Serial.print(control_get_left_ref(), 3);
        Serial.print(" | L vel: ");
        Serial.print(encoders_get_left_rad_s(), 3);
        Serial.print(" | L u: ");
        Serial.print(control_get_left_u(), 2);

        Serial.print(" || R ref: ");
        Serial.print(control_get_right_ref(), 3);
        Serial.print(" | R vel: ");
        Serial.print(encoders_get_right_rad_s(), 3);
        Serial.print(" | R u: ");
        Serial.println(control_get_right_u(), 2);
    }

    delay(5);
}