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

    delay(300);
    return true;
}

void oled_show_message(
    const char *title,
    const char *line1,
    const char *line2,
    const char *line3,
    const char *line4
)
{
    if (!oled_ok) return;

    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.println(title);
    display.println("----------------");

    if (line1 && strlen(line1) > 0) display.println(line1);
    if (line2 && strlen(line2) > 0) display.println(line2);
    if (line3 && strlen(line3) > 0) display.println(line3);
    if (line4 && strlen(line4) > 0) display.println(line4);

    display.display();
}

void oled_show_boot_status(
    bool bno_ok,
    bool center_ok,
    bool left_ok,
    bool right_ok,
    bool wifi_ok,
    bool uros_ok
)
{
    if (!oled_ok) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.println("Boot d3im3r");
    display.print("BNO......");
    display.println(bno_ok ? "OK" : "--");

    display.print("CENTRO...");
    display.println(center_ok ? "OK" : "--");

    display.print("LEFT.....");
    display.println(left_ok ? "OK" : "--");

    display.print("RIGHT....");
    display.println(right_ok ? "OK" : "--");

    display.print("WIFI.....");
    display.println(wifi_ok ? "OK" : "--");

    display.print("uROS.....");
    display.println(uros_ok ? "OK" : "--");

    display.display();
}

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
)
{
    if (!oled_ok) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print("C:");
    display.print(d_center_m, 2);
    display.print(" L:");
    display.print(d_left_m, 2);
    display.print(" R:");
    display.println(d_right_m, 2);

    display.print("Y:");
    if (yaw_ok) {
        display.println(yaw_deg, 1);
    } else {
        display.println("--");
    }

    display.print("WL:");
    display.print(vel_left_rad_s, 2);
    display.print(" WR:");
    display.println(vel_right_rad_s, 2);

    display.print("Cal:");
    display.println(calibration_status);

    display.print("STOP:");
    display.println(robot_stopped ? "YES" : "NO");

    display.display();
}