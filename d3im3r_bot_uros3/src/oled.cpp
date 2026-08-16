#include "oled.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

static Adafruit_SH1106G display(128, 64, &Wire, -1);
static bool oled_ok = false;

bool oled_init()
{
    oled_ok = display.begin(0x3C, true);

    if (!oled_ok) {
        return false;
    }

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OLED OK");
    display.display();

    delay(400);
    return true;
}

void oled_show_boot(
    OledBootStage stage,
    bool bno_ok,
    bool tof_right_ok,
    bool tof_left_ok,
    bool tof_center_ok,
    bool uros_ok,
    const char* msg
)
{
    if (!oled_ok) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.println("Boot d3im3r");

    display.print("Etapa: ");
    switch(stage)
    {
        case OLED_BOOT_START:      display.println("START"); break;
        case OLED_BOOT_I2C:        display.println("I2C"); break;
        case OLED_BOOT_BNO:        display.println("BNO055"); break;
        case OLED_BOOT_TOF_RIGHT:  display.println("TOF RIGHT"); break;
        case OLED_BOOT_TOF_LEFT:   display.println("TOF LEFT"); break;
        case OLED_BOOT_TOF_CENTER: display.println("TOF CENTER"); break;
        case OLED_BOOT_UROS:       display.println("microROS"); break;
        case OLED_BOOT_READY:      display.println("READY"); break;
        case OLED_BOOT_ERROR:      display.println("ERROR"); break;
        default:                   display.println("--"); break;
    }

    display.print("BNO: ");
    display.println(bno_ok ? "OK" : "--");

    display.print("R:");
    display.print(tof_right_ok ? "Y " : "N ");
    display.print("L:");
    display.print(tof_left_ok ? "Y " : "N ");
    display.print("C:");
    display.println(tof_center_ok ? "Y" : "N");

    display.print("uROS: ");
    display.println(uros_ok ? "OK" : "--");

    if (msg != nullptr) {
        display.println(msg);
    }

    display.display();
}

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
)
{
    if (!oled_ok) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    // Línea 1: centro arriba
    display.print("C: ");
    if (center_ok) {
        display.print(d_center_m, 3);
        display.println(" m");
    } else {
        display.println("--");
    }

    // Línea 2: left y right abajo
    display.print("L: ");
    if (left_ok) {
        display.print(d_left_m, 3);
    } else {
        display.print("--");
    }

    display.print("  R: ");
    if (right_ok) {
        display.println(d_right_m, 3);
    } else {
        display.println("--");
    }

    // Línea 3: yaw
    display.print("Yaw: ");
    if (yaw_ok) {
        display.print(yaw_deg, 1);
        display.println(" deg");
    } else {
        display.println("--");
    }
    
    // Línea 4
    display.println("--------------------");
    // Línea 5: calibración IMU
    display.print("Cal S");
    display.print(cal_sys);
    display.print(" G");
    display.print(cal_gyro);
    display.print(" A");
    display.print(cal_accel);
    display.print(" M");
    display.println(cal_mag);

    // Línea 6: validación TOF
    display.print("TOF C:");
    display.print(center_ok ? "Y " : "N ");
    display.print("L:");
    display.print(left_ok ? "Y " : "N ");
    display.print("R:");
    display.println(right_ok ? "Y" : "N");

    display.display();
}