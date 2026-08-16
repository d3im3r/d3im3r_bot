#include "sensors.h"
#include "app_config.h"

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>
#include <math.h>

static Adafruit_BNO055 bno(55, 0x28);

static Adafruit_VL53L0X sensor_center;
static Adafruit_VL53L0X sensor_right;
static Adafruit_VL53L0X sensor_left;

// Datos exportados
float imu_qx = 0.0f;
float imu_qy = 0.0f;
float imu_qz = 0.0f;
float imu_qw = 1.0f;
float imu_yaw = 0.0f;

uint16_t calibration_status = 0;
uint8_t cal_sys = 0;
uint8_t cal_gyro = 0;
uint8_t cal_accel = 0;
uint8_t cal_mag = 0;

float tof_distances[3] = {-1.0f, -1.0f, -1.0f};

bool bno_ok = false;
bool tof_center_ok = false;
bool tof_left_ok = false;
bool tof_right_ok = false;

struct ToFConfig {
    Adafruit_VL53L0X &sensor;
    int xshut_pin;
    uint8_t addr;
    const char *label;
    bool &status_flag;
};

static bool i2c_device_present(uint8_t address)
{
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}

static bool wait_for_i2c_device(uint8_t address, uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
        if (i2c_device_present(address)) return true;
        delay(10);
    }
    return false;
}

static bool init_tof(Adafruit_VL53L0X &sensor, int xshut_pin, uint8_t new_addr, const char *label)
{
    digitalWrite(xshut_pin, HIGH);
    delay(100);

    if (!wait_for_i2c_device(0x29, 600) || !sensor.begin()) {
        Serial.print("[TOF] ERROR sensor ");
        Serial.println(label);
        return false;
    }

    sensor.setAddress(new_addr);
    Serial.print("[TOF] Direccion ");
    Serial.print(label);
    Serial.print(" -> 0x");
    Serial.println(new_addr, HEX);
    return true;
}

bool sensors_init()
{
    Serial.println("[SENSORS] Iniciando BNO055...");
    bno_ok = bno.begin();

    if (!bno_ok) {
        Serial.println("[SENSORS] ERROR BNO055");
    } else {
        delay(1000);
        bno.setExtCrystalUse(true);
        Serial.println("[SENSORS] BNO055 OK");
    }

    Serial.println("[SENSORS] Inicializacion VL53L0X x3");

    ToFConfig tof_sensors[] = {
        { sensor_right,  XSHUT_RIGHT_PIN,  TOF_ADDR_RIGHT,  "DERECHO",   tof_right_ok },
        { sensor_left,   XSHUT_LEFT_PIN,   TOF_ADDR_LEFT,   "IZQUIERDO", tof_left_ok },
        { sensor_center, XSHUT_CENTER_PIN, TOF_ADDR_CENTER, "CENTRAL",   tof_center_ok }
    };

    for (auto &cfg : tof_sensors) {
        pinMode(cfg.xshut_pin, OUTPUT);
        digitalWrite(cfg.xshut_pin, LOW);
    }
    delay(200);

    for (auto &cfg : tof_sensors) {
        Serial.print("[TOF] Activando "); Serial.println(cfg.label);
        cfg.status_flag = init_tof(cfg.sensor, cfg.xshut_pin, cfg.addr, cfg.label);
        delay(100);
    }

    Serial.println("[TOF] Sensores listos");
    return (bno_ok || tof_center_ok || tof_left_ok || tof_right_ok);
}

void sensors_update_imu()
{
    if (!bno_ok) return;

    imu::Quaternion q = bno.getQuat();
    imu_qx = q.x();
    imu_qy = q.y();
    imu_qz = q.z();
    imu_qw = q.w();

    imu_yaw = atan2(
        2.0f * (imu_qw * imu_qz + imu_qx * imu_qy),
        1.0f - 2.0f * (imu_qy * imu_qy + imu_qz * imu_qz)
    );

    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);
    calibration_status = static_cast<uint16_t>(cal_sys * 1000 + cal_gyro * 100 + cal_accel * 10 + cal_mag);
}

static float read_tof_m(Adafruit_VL53L0X &sensor, bool sensor_ok)
{
    if (!sensor_ok) return -1.0f;

    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    return (measure.RangeStatus == 4) ? -1.0f : (measure.RangeMilliMeter / 1000.0f);
}

void sensors_update_tof()
{
    tof_distances[0] = read_tof_m(sensor_center, tof_center_ok);
    tof_distances[1] = read_tof_m(sensor_left, tof_left_ok);
    tof_distances[2] = read_tof_m(sensor_right, tof_right_ok);
}